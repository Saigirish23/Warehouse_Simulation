#!/usr/bin/env python3
"""
Emergency Stop System for Competition Robot

COMPETITION REQUIREMENT:
- Emergency stop response: <500ms
- Monitor hardware e-stop button
- Override Nav2 commands
- Log all stop events

This node monitors a GPIO-based emergency stop button and immediately
stops the robot by publishing zero velocity commands.

Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
from typing import Optional

# GPIO support (uncomment when running on real hardware)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class EmergencyStopNode(Node):
    """
    Emergency stop system with <500ms response time.
    
    Monitors GPIO pin for e-stop button and immediately halts robot motion.
    Higher priority than Nav2 by publishing directly to /cmd_vel.
    """
    
    def __init__(self):
        super().__init__('emergency_stop')
        
        # Parameters
        self.declare_parameter('estop_gpio_pin', 17)        # BCM pin 17
        self.declare_parameter('monitor_rate', 50.0)        # 50 Hz = 20ms check interval
        self.declare_parameter('use_gpio', False)           # Set True for real hardware
        self.declare_parameter('active_low', True)          # Button pulls pin LOW when pressed
        
        self.gpio_pin = self.get_parameter('estop_gpio_pin').value
        self.use_gpio = self.get_parameter('use_gpio').value
        self.active_low = self.get_parameter('active_low').value
        
        # State variables
        self.estop_active: bool = False
        self.last_check_time: float = time.time()
        self.stop_count: int = 0
        
        # Publishers - HIGH PRIORITY
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10  # Higher QoS for priority
        )
        
        self.estop_status_pub = self.create_publisher(
            Bool,
            '/emergency_stop/status',
            10
        )
        
        # Subscriber for software-triggered e-stop
        self.software_estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop/trigger',
            self.software_estop_callback,
            10
        )
        
        # Initialize GPIO if available and requested
        if self.use_gpio and GPIO_AVAILABLE:
            self.init_gpio()
            self.get_logger().info(f'GPIO emergency stop initialized on pin {self.gpio_pin}')
        else:
            if self.use_gpio:
                self.get_logger().warn('GPIO requested but not available - using software trigger only')
            else:
                self.get_logger().info('Running in software e-stop mode (no GPIO)')
        
        # High-frequency timer for <500ms response
        check_period = 1.0 / self.get_parameter('monitor_rate').value
        self.check_timer = self.create_timer(check_period, self.check_estop)
        
        self.get_logger().info('Emergency Stop System ready')
        self.get_logger().info(f'Response time: <{check_period*1000:.1f}ms per check')
        
    def init_gpio(self) -> None:
        """Initialize GPIO pin for emergency stop button."""
        try:
            GPIO.setmode(GPIO.BCM)
            
            if self.active_low:
                # Button connects pin to GND when pressed
                GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            else:
                # Button connects pin to 3.3V when pressed
                GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                
            self.get_logger().info(f'GPIO pin {self.gpio_pin} configured')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            self.use_gpio = False
    
    def check_estop(self) -> None:
        """
        Check emergency stop status and respond immediately.
        
        CRITICAL: This runs at 50 Hz (20ms intervals) to ensure
        <500ms response time as per competition requirements.
        """
        current_time = time.time()
        response_time = (current_time - self.last_check_time) * 1000  # ms
        self.last_check_time = current_time
        
        # Check GPIO state
        estop_triggered = False
        
        if self.use_gpio and GPIO_AVAILABLE:
            try:
                gpio_state = GPIO.input(self.gpio_pin)
                
                if self.active_low:
                    estop_triggered = (gpio_state == GPIO.LOW)
                else:
                    estop_triggered = (gpio_state == GPIO.HIGH)
                    
            except Exception as e:
                self.get_logger().error(f'GPIO read error: {e}', throttle_duration_sec=1.0)
        
        # Update state and take action
        if estop_triggered and not self.estop_active:
            # E-stop activated
            self.activate_estop(response_time)
            
        elif not estop_triggered and self.estop_active:
            # E-stop released
            self.deactivate_estop()
        
        elif self.estop_active:
            # E-stop still active - keep sending stop commands
            self.send_stop_command()
        
        # Publish status
        self.publish_status()
    
    def activate_estop(self, response_time: float) -> None:
        """
        Activate emergency stop and halt robot.
        
        Args:
            response_time: Time taken to detect e-stop in milliseconds
        """
        self.estop_active = True
        self.stop_count += 1
        
        # Immediately stop robot
        self.send_stop_command()
        
        # Log event with timestamp
        self.get_logger().error(
            f'EMERGENCY STOP ACTIVATED! '
            f'Response time: {response_time:.1f}ms '
            f'(Event #{self.stop_count})'
        )
        
        # Competition requirement: Verify <500ms response
        if response_time > 500:
            self.get_logger().fatal(
                f'COMPETITION VIOLATION: Response time {response_time:.1f}ms exceeds 500ms limit!'
            )
    
    def deactivate_estop(self) -> None:
        """Deactivate emergency stop and resume normal operation."""
        self.estop_active = False
        
        self.get_logger().warn('Emergency stop RELEASED - robot can resume motion')
    
    def send_stop_command(self) -> None:
        """
        Publish zero velocity command to immediately stop robot.
        
        This overrides any Nav2 commands by publishing at higher frequency.
        """
        stop_msg = Twist()
        # All fields default to 0.0
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(stop_msg)
    
    def software_estop_callback(self, msg: Bool) -> None:
        """
        Handle software-triggered emergency stop.
        
        Args:
            msg: Bool message - True to activate, False to deactivate
        """
        if msg.data and not self.estop_active:
            self.get_logger().warn('Software emergency stop triggered')
            self.activate_estop(0.0)  # Immediate software trigger
            
        elif not msg.data and self.estop_active:
            self.get_logger().warn('Software emergency stop released')
            self.deactivate_estop()
    
    def publish_status(self) -> None:
        """Publish current e-stop status for monitoring."""
        status_msg = Bool()
        status_msg.data = self.estop_active
        self.estop_status_pub.publish(status_msg)
    
    def cleanup(self) -> None:
        """Clean up GPIO resources."""
        if self.use_gpio and GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
                self.get_logger().info('GPIO cleaned up')
            except Exception as e:
                self.get_logger().error(f'GPIO cleanup error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = EmergencyStopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
