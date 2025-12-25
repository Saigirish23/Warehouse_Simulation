#!/usr/bin/env python3
"""
WiFi Connection Monitor for Competition Robot

COMPETITION REQUIREMENT:
- Robot must stop if WiFi connection lost
- Ping router/edge server every 2 seconds
- Failsafe after 3 consecutive ping failures
- Publish system status for monitoring

Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import subprocess
import time
from typing import Optional
import json


class WiFiMonitor(Node):
    """
    Monitor WiFi connectivity and trigger failsafe if connection lost.
    
    Competition Requirements:
    - Ping every 2 seconds
    - 3 consecutive failures trigger emergency stop
    - Cancel active navigation goals
    - Log all connectivity events
    """
    
    def __init__(self):
        super().__init__('wifi_monitor')
        
        # Parameters
        self.declare_parameter('target_host', '192.168.1.1')  # Router IP or edge server
        self.declare_parameter('ping_interval', 2.0)          # 2 seconds
        self.declare_parameter('failure_threshold', 3)        # 3 consecutive failures
        self.declare_parameter('ping_timeout', 1.0)           # 1 second timeout per ping
        self.declare_parameter('enable_failsafe', True)       # Enable emergency stop
        
        self.target_host = self.get_parameter('target_host').value
        self.ping_interval = self.get_parameter('ping_interval').value
        self.failure_threshold = self.get_parameter('failure_threshold').value
        self.ping_timeout = self.get_parameter('ping_timeout').value
        self.enable_failsafe = self.get_parameter('enable_failsafe').value
        
        # State variables
        self.consecutive_failures: int = 0
        self.total_pings: int = 0
        self.failed_pings: int = 0
        self.failsafe_active: bool = False
        self.last_ping_success: bool = True
        self.start_time: float = time.time()
        
        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        self.failsafe_pub = self.create_publisher(
            Bool,
            '/wifi_monitor/failsafe_active',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Action client for canceling Nav2 goals
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Timer for periodic pings
        self.ping_timer = self.create_timer(
            self.ping_interval,
            self.check_connectivity
        )
        
        self.get_logger().info('WiFi Monitor initialized')
        self.get_logger().info(f'Monitoring: {self.target_host} every {self.ping_interval}s')
        self.get_logger().info(f'Failsafe threshold: {self.failure_threshold} consecutive failures')
    
    def ping_host(self) -> bool:
        """
        Ping target host to check connectivity.
        
        Returns:
            True if ping successful, False otherwise
        """
        try:
            # Use ping command with timeout
            result = subprocess.run(
                ['ping', '-c', '1', '-W', str(int(self.ping_timeout)), self.target_host],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.ping_timeout + 0.5
            )
            
            return result.returncode == 0
            
        except subprocess.TimeoutExpired:
            return False
        except Exception as e:
            self.get_logger().error(f'Ping error: {e}')
            return False
    
    def check_connectivity(self) -> None:
        """
        Periodically check WiFi connectivity and trigger failsafe if needed.
        
        COMPETITION REQUIREMENT:
        - Monitor every 2 seconds
        - Failsafe after 3 consecutive failures
        """
        self.total_pings += 1
        ping_success = self.ping_host()
        
        if ping_success:
            # Ping successful
            if self.consecutive_failures > 0:
                self.get_logger().info('WiFi connection restored')
            
            self.consecutive_failures = 0
            self.last_ping_success = True
            
            # Deactivate failsafe if it was active
            if self.failsafe_active:
                self.deactivate_failsafe()
        else:
            # Ping failed
            self.consecutive_failures += 1
            self.failed_pings += 1
            self.last_ping_success = False
            
            self.get_logger().warn(
                f'WiFi ping failed ({self.consecutive_failures}/{self.failure_threshold})'
            )
            
            # Check if threshold reached
            if self.consecutive_failures >= self.failure_threshold and not self.failsafe_active:
                self.activate_failsafe()
        
        # Publish status
        self.publish_status()
    
    def activate_failsafe(self) -> None:
        """
        Activate failsafe mode due to WiFi loss.
        
        COMPETITION REQUIREMENT:
        - Stop robot immediately
        - Cancel navigation goals
        - Log event with timestamp
        """
        self.failsafe_active = True
        
        self.get_logger().error(
            f'FAILSAFE ACTIVATED: WiFi connection lost after '
            f'{self.consecutive_failures} consecutive failures'
        )
        
        if self.enable_failsafe:
            # Cancel active Nav2 goals
            self.cancel_navigation_goal()
            
            # Stop robot immediately
            self.stop_robot()
            
            # Log event
            self.log_failsafe_event()
        
        # Publish failsafe status
        failsafe_msg = Bool()
        failsafe_msg.data = True
        self.failsafe_pub.publish(failsafe_msg)
    
    def deactivate_failsafe(self) -> None:
        """Deactivate failsafe when connection restored."""
        self.failsafe_active = False
        
        self.get_logger().warn('Failsafe deactivated - connection restored')
        
        # Publish status
        failsafe_msg = Bool()
        failsafe_msg.data = False
        self.failsafe_pub.publish(failsafe_msg)
    
    def cancel_navigation_goal(self) -> None:
        """Cancel any active Nav2 navigation goals."""
        try:
            if self.nav_client.server_is_ready():
                # Cancel all goals
                future = self.nav_client._cancel_goal_async(None)
                self.get_logger().info('Navigation goal cancellation requested')
            else:
                self.get_logger().warn('Nav2 action server not available')
        except Exception as e:
            self.get_logger().error(f'Failed to cancel navigation goal: {e}')
    
    def stop_robot(self) -> None:
        """Immediately stop robot by publishing zero velocity."""
        stop_msg = Twist()
        # All velocities default to 0.0
        
        # Send multiple stop commands to ensure it takes effect
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)
        
        self.get_logger().warn('Robot stopped due to WiFi loss')
    
    def log_failsafe_event(self) -> None:
        """Log failsafe activation event to file."""
        try:
            event_data = {
                'timestamp': time.time(),
                'uptime_seconds': time.time() - self.start_time,
                'consecutive_failures': self.consecutive_failures,
                'total_pings': self.total_pings,
                'failed_pings': self.failed_pings,
                'target_host': self.target_host
            }
            
            # Append to log file
            with open('/tmp/wifi_failsafe_log.json', 'a') as f:
                f.write(json.dumps(event_data) + '\n')
                
        except Exception as e:
            self.get_logger().error(f'Failed to log failsafe event: {e}')
    
    def publish_status(self) -> None:
        """
        Publish current WiFi and system status.
        
        Status includes connectivity state, statistics, and failsafe status.
        """
        uptime = time.time() - self.start_time
        success_rate = ((self.total_pings - self.failed_pings) / self.total_pings * 100) \
                       if self.total_pings > 0 else 100.0
        
        status_data = {
            'node': 'wifi_monitor',
            'connected': self.last_ping_success,
            'failsafe_active': self.failsafe_active,
            'consecutive_failures': self.consecutive_failures,
            'total_pings': self.total_pings,
            'failed_pings': self.failed_pings,
            'success_rate_percent': round(success_rate, 1),
            'uptime_seconds': round(uptime, 1),
            'target_host': self.target_host
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
        
        # Log statistics periodically
        if self.total_pings % 30 == 0:  # Every ~60 seconds
            self.get_logger().info(
                f'WiFi Stats: {self.total_pings} pings, '
                f'{success_rate:.1f}% success, '
                f'uptime: {uptime:.0f}s',
                throttle_duration_sec=60.0
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = WiFiMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
