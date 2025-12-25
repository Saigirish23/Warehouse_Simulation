#!/usr/bin/env python3
"""
Obstacle-based Dynamic Speed Controller for Competition Robot

COMPETITION REQUIREMENT:
- Max speed in open space: 1.0 m/s
- Max speed near obstacles (<15cm): 0.3 m/s
- Linear interpolation between 0.15m and 5m

This node monitors LiDAR scan data and dynamically adjusts Nav2 velocity limits
based on the distance to the nearest obstacle.

Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import math
from typing import Optional


class ObstacleSpeedController(Node):
    """
    Dynamically adjusts robot speed based on obstacle proximity.
    
    Competition Requirements:
    - >5m: 1.0 m/s (full speed)
    - 1-5m: 0.5-1.0 m/s (linear interpolation)
    - 0.15-1m: 0.3-0.5 m/s (slow approach)
    - <0.15m: 0.3 m/s (minimum safe speed)
    """
    
    def __init__(self):
        super().__init__('obstacle_speed_controller')
        
        # Competition parameters
        self.declare_parameter('max_speed_open', 1.0)           # Full speed
        self.declare_parameter('max_speed_near_obstacle', 0.3)  # Safety speed
        self.declare_parameter('safe_distance_threshold', 0.15) # 15cm danger zone
        self.declare_parameter('full_speed_distance', 5.0)      # 5m clear zone
        self.declare_parameter('update_rate', 10.0)             # 10 Hz updates
        self.declare_parameter('scan_angle_range', 60.0)        # ±60° front cone
        
        # Get parameters
        self.max_speed_open = self.get_parameter('max_speed_open').value
        self.max_speed_near = self.get_parameter('max_speed_near_obstacle').value
        self.safe_threshold = self.get_parameter('safe_distance_threshold').value
        self.full_speed_dist = self.get_parameter('full_speed_distance').value
        self.scan_angle = math.radians(self.get_parameter('scan_angle_range').value)
        
        # State variables
        self.current_min_distance: Optional[float] = None
        self.current_speed_limit: float = self.max_speed_open
        self.last_update_time = self.get_clock().now()
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Service client for updating Nav2 parameters
        self.param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters'
        )
        
        # Timer for periodic parameter updates
        update_period = 1.0 / self.get_parameter('update_rate').value
        self.update_timer = self.create_timer(update_period, self.update_speed_limits)
        
        # Status publisher for monitoring
        self.status_pub = self.create_publisher(
            TwistStamped,
            '/speed_controller/status',
            10
        )
        
        self.get_logger().info('Obstacle Speed Controller initialized')
        self.get_logger().info(f'Speed limits: {self.max_speed_near} m/s (near) to {self.max_speed_open} m/s (open)')
        
    def scan_callback(self, msg: LaserScan) -> None:
        """
        Process LiDAR scan and find minimum distance to obstacles in front arc.
        
        Args:
            msg: LaserScan message from /scan topic
        """
        if len(msg.ranges) == 0:
            return
            
        # Calculate front cone indices
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)
        
        # Find indices for ±scan_angle from front (0 degrees)
        front_start_angle = -self.scan_angle / 2
        front_end_angle = self.scan_angle / 2
        
        valid_distances = []
        
        for i, distance in enumerate(msg.ranges):
            # Calculate angle for this reading
            angle = angle_min + (i * angle_increment)
            
            # Check if within front cone
            if front_start_angle <= angle <= front_end_angle:
                # Filter invalid readings
                if msg.range_min < distance < msg.range_max and not math.isinf(distance):
                    valid_distances.append(distance)
        
        # Update minimum distance
        if valid_distances:
            self.current_min_distance = min(valid_distances)
        else:
            # No valid readings - assume clear
            self.current_min_distance = self.full_speed_dist
            
    def calculate_speed_limit(self, min_distance: float) -> float:
        """
        Calculate appropriate speed limit based on obstacle distance.
        
        Competition speed profile:
        - d < 0.15m: 0.3 m/s (danger zone)
        - 0.15m ≤ d < 1.0m: 0.3 - 0.5 m/s (slow approach)
        - 1.0m ≤ d < 5.0m: 0.5 - 1.0 m/s (moderate speed)
        - d ≥ 5.0m: 1.0 m/s (full speed)
        
        Args:
            min_distance: Distance to nearest obstacle in meters
            
        Returns:
            Appropriate speed limit in m/s
        """
        if min_distance < self.safe_threshold:
            # Danger zone - enforce minimum speed
            return self.max_speed_near
        elif min_distance < 1.0:
            # Slow approach zone (0.15m - 1.0m)
            # Linear interpolation: 0.3 m/s → 0.5 m/s
            ratio = (min_distance - self.safe_threshold) / (1.0 - self.safe_threshold)
            return 0.3 + (ratio * 0.2)  # 0.3 to 0.5
        elif min_distance < self.full_speed_dist:
            # Moderate speed zone (1.0m - 5.0m)
            # Linear interpolation: 0.5 m/s → 1.0 m/s
            ratio = (min_distance - 1.0) / (self.full_speed_dist - 1.0)
            return 0.5 + (ratio * 0.5)  # 0.5 to 1.0
        else:
            # Clear zone - full speed
            return self.max_speed_open
    
    def update_speed_limits(self) -> None:
        """
        Periodically update Nav2 controller speed limits based on obstacle distance.
        """
        if self.current_min_distance is None:
            return
            
        # Calculate new speed limit
        new_speed_limit = self.calculate_speed_limit(self.current_min_distance)
        
        # Only update if change is significant (>5% difference)
        if abs(new_speed_limit - self.current_speed_limit) > 0.05:
            self.current_speed_limit = new_speed_limit
            
            # Update Nav2 controller parameters
            if self.param_client.service_is_ready():
                self.set_nav2_speed_limit(new_speed_limit)
            
            self.get_logger().info(
                f'Speed limit updated: {new_speed_limit:.2f} m/s '
                f'(obstacle at {self.current_min_distance:.2f}m)',
                throttle_duration_sec=2.0
            )
        
        # Publish status for monitoring
        self.publish_status()
    
    def set_nav2_speed_limit(self, speed_limit: float) -> None:
        """
        Update Nav2 controller_server velocity parameters.
        
        Args:
            speed_limit: New maximum velocity in m/s
        """
        try:
            request = SetParameters.Request()
            
            # Update max_vel_x parameter
            param_max_vel_x = Parameter()
            param_max_vel_x.name = 'FollowPath.max_vel_x'
            param_max_vel_x.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=speed_limit
            )
            
            # Update max_speed_xy parameter
            param_max_speed = Parameter()
            param_max_speed.name = 'FollowPath.max_speed_xy'
            param_max_speed.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=speed_limit
            )
            
            request.parameters = [param_max_vel_x, param_max_speed]
            
            # Send async request
            future = self.param_client.call_async(request)
            
        except Exception as e:
            self.get_logger().error(f'Failed to update Nav2 parameters: {e}')
    
    def publish_status(self) -> None:
        """
        Publish current speed controller status for monitoring.
        """
        status_msg = TwistStamped()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = 'base_link'
        
        # Use twist fields to encode status
        status_msg.twist.linear.x = self.current_speed_limit  # Current limit
        status_msg.twist.linear.y = self.current_min_distance if self.current_min_distance else -1.0
        status_msg.twist.linear.z = self.max_speed_open  # Max possible
        
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObstacleSpeedController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
