#!/usr/bin/env python3
"""
Waypoint Validator for Geotag-Based Scanning

Used during teleop mapping phase to validate and record optimal scanning positions.

COMPETITION REQUIREMENTS:
- Front wheels 15-25cm from rack (accounting for wheelbase offset)
- Robot parallel to rack (±5° tolerance)
- Robot within shelf lateral bounds (9.1m)
- No obstacles within 15cm

Usage during mapping:
  ros2 run scuttle_navigation2 waypoint_validator.py --save R01

Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import yaml
import math
import numpy as np
import argparse
from typing import Optional, Dict, Tuple
from dataclasses import dataclass, asdict
import os


@dataclass
class ValidationMetadata:
    """Validation metadata for a scanning waypoint."""
    rack_id: str
    x: float
    y: float
    yaw: float
    front_wheel_distance_cm: float  # Distance from front wheels to rack
    parallel_error_deg: float        # Angular alignment error
    shelf_lateral_position_m: float  # Position along shelf width
    obstacles_clear: bool            # No obstacles within 15cm
    validation_timestamp: str
    validated_by: str = "mapping_operator"
    notes: str = ""


class WaypointValidator(Node):
    """
    Validates robot position for optimal QR scanning during mapping.
    
    Accounts for wheelbase offset to measure from FRONT WHEELS, not base_link.
    """
    
    def __init__(self):
        super().__init__('waypoint_validator')
        
        # Parameters
        self.declare_parameter('wheelbase_offset_m', 0.20)  # Distance base_link to front wheels
        self.declare_parameter('min_wheel_distance_cm', 15.0)
        self.declare_parameter('max_wheel_distance_cm', 25.0)
        self.declare_parameter('parallel_tolerance_deg', 5.0)
        self.declare_parameter('obstacle_clearance_cm', 15.0)
        self.declare_parameter('shelf_width_m', 9.1)
        self.declare_parameter('waypoints_file', 
                             '/home/b24me1066/Warehouse_Simulation/config/waypoints_competition.yaml')
        
        # Get parameters
        self.wheelbase_offset = self.get_parameter('wheelbase_offset_m').value
        self.min_wheel_dist = self.get_parameter('min_wheel_distance_cm').value
        self.max_wheel_dist = self.get_parameter('max_wheel_distance_cm').value
        self.parallel_tol = self.get_parameter('parallel_tolerance_deg').value
        self.obstacle_clearance = self.get_parameter('obstacle_clearance_cm').value
        self.shelf_width = self.get_parameter('shelf_width_m').value
        self.waypoints_file = self.get_parameter('waypoints_file').value
        
        # State
        self.current_pose: Optional[PoseWithCovarianceStamped] = None
        self.latest_scan: Optional[LaserScan] = None
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Waypoint Validator ready')
        self.get_logger().info(f'Wheelbase offset: {self.wheelbase_offset}m (base_link → front wheels)')
        self.get_logger().info(f'Valid wheel distance: {self.min_wheel_dist}-{self.max_wheel_dist}cm from rack')
        self.get_logger().info(f'Parallel tolerance: ±{self.parallel_tol}°')
        
    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Store current robot pose."""
        self.current_pose = msg
    
    def scan_callback(self, msg: LaserScan) -> None:
        """Store latest laser scan."""
        self.latest_scan = msg
    
    def get_current_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current robot position (x, y, yaw).
        
        Uses TF if AMCL pose not available (SLAM mapping phase).
        
        Returns:
            Tuple of (x, y, yaw) or None if not available
        """
        # Try AMCL pose first (Phase 2: Navigation)
        if self.current_pose is not None:
            pose = self.current_pose.pose.pose
            
            # Extract yaw from quaternion
            quat = pose.orientation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (pose.position.x, pose.position.y, yaw)
        
        # Fallback to TF (Phase 1: SLAM mapping)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            quat = transform.transform.rotation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (x, y, yaw)
            
        except Exception as e:
            # Silent fail during normal operation, only log in debug
            pass
        
        return None
    
    def calculate_front_wheel_position(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        """
        Calculate front wheel center position from base_link.
        
        Args:
            x, y: Base_link position
            yaw: Robot orientation
            
        Returns:
            (front_x, front_y) coordinates
        """
        # Front wheels are wheelbase_offset forward from base_link
        front_x = x + self.wheelbase_offset * math.cos(yaw)
        front_y = y + self.wheelbase_offset * math.sin(yaw)
        
        return (front_x, front_y)
    
    def measure_distance_to_rack(self) -> Optional[float]:
        """
        Measure distance from front wheels to rack in front.
        
        IMPORTANT: lidar_1 is rotated 180° in the robot frame!
        - angle_min=0 points BACKWARD
        - angle ~π (180°) points FORWARD
        
        Uses laser scan readings in front ±15° cone.
        
        Returns:
            Distance in meters, or None if no valid readings
        """
        if self.latest_scan is None:
            return None
        
        # Since lidar is rotated 180°, front is at angle ~π (180°)
        center_angle = math.pi
        angle_range = math.radians(15)
        front_readings = []
        
        for i, distance in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
            
            # Check if angle is within ±15° of π (forward direction)
            angle_diff = abs(angle - center_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff <= angle_range:
                if (self.latest_scan.range_min < distance < self.latest_scan.range_max 
                    and not math.isinf(distance)):
                    front_readings.append(distance)
        
        if not front_readings:
            return None
        
        # Lidar reading + lidar offset (lidar is 2.7cm forward of base_link)
        lidar_distance = min(front_readings)
        lidar_offset = 0.027
        base_link_distance = lidar_distance + lidar_offset
        
        # Adjust for wheelbase offset - front wheels are FURTHER from rack
        front_wheel_distance = base_link_distance - self.wheelbase_offset
        
        return front_wheel_distance
    
    def estimate_rack_orientation(self) -> Optional[float]:
        """
        Estimate rack orientation based on laser scan pattern.
        
        Assumes rack is a flat surface in front of robot.
        
        Returns:
            Rack orientation angle, or None if cannot determine
        """
        if self.latest_scan is None:
            return None
        
        # Get readings in front ±45° cone
        angle_range = math.radians(45)
        readings = []
        
        for i, distance in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
            
            if abs(angle) <= angle_range:
                if (self.latest_scan.range_min < distance < self.latest_scan.range_max 
                    and not math.isinf(distance)):
                    readings.append((angle, distance))
        
        if len(readings) < 3:
            return None
        
        # Fit line to points - rack should be roughly linear
        angles = np.array([r[0] for r in readings])
        distances = np.array([r[1] for r in readings])
        
        # Convert to Cartesian
        x_points = distances * np.cos(angles)
        y_points = distances * np.sin(angles)
        
        # Fit line
        if len(x_points) > 0:
            coeffs = np.polyfit(x_points, y_points, 1)
            rack_angle = math.atan(coeffs[0])
            return rack_angle
        
        return None
    
    def check_parallel_alignment(self, robot_yaw: float) -> Tuple[bool, float]:
        """
        Check if robot is parallel to rack.
        
        Args:
            robot_yaw: Current robot orientation
            
        Returns:
            (is_parallel, error_degrees)
        """
        rack_orientation = self.estimate_rack_orientation()
        
        if rack_orientation is None:
            return (False, 999.0)
        
        # Calculate alignment error
        error_rad = abs(robot_yaw - rack_orientation)
        
        # Normalize to [-pi, pi]
        while error_rad > math.pi:
            error_rad -= 2 * math.pi
        error_rad = abs(error_rad)
        
        error_deg = math.degrees(error_rad)
        is_parallel = error_deg <= self.parallel_tol
        
        return (is_parallel, error_deg)
    
    def check_obstacle_clearance(self) -> Tuple[bool, float]:
        """
        Check for obstacles within clearance distance.
        
        Returns:
            (is_clear, minimum_distance_cm)
        """
        if self.latest_scan is None:
            return (False, 0.0)
        
        # Check all 360° for obstacles
        min_distance = float('inf')
        
        for distance in self.latest_scan.ranges:
            if (self.latest_scan.range_min < distance < self.latest_scan.range_max 
                and not math.isinf(distance)):
                min_distance = min(min_distance, distance)
        
        min_distance_cm = min_distance * 100
        is_clear = min_distance_cm >= self.obstacle_clearance
        
        return (is_clear, min_distance_cm)
    
    def validate_current_position(self, rack_id: str) -> Optional[ValidationMetadata]:
        """
        Validate current robot position for scanning.
        
        Args:
            rack_id: Rack identifier (e.g., 'R01')
            
        Returns:
            ValidationMetadata if position valid, None otherwise
        """
        # Get current position
        position = self.get_current_position()
        if position is None:
            self.get_logger().error('No robot position available')
            return None
        
        x, y, yaw = position
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'VALIDATING POSITION FOR RACK {rack_id}')
        self.get_logger().info('=' * 60)
        
        # Measure distance from front wheels to rack
        wheel_distance = self.measure_distance_to_rack()
        if wheel_distance is None:
            self.get_logger().error('Cannot measure distance to rack')
            return None
        
        wheel_distance_cm = wheel_distance * 100
        distance_ok = self.min_wheel_dist <= wheel_distance_cm <= self.max_wheel_dist
        
        self.get_logger().info(f'Front wheel distance: {wheel_distance_cm:.1f}cm (target: {self.min_wheel_dist}-{self.max_wheel_dist}cm)')
        if distance_ok:
            self.get_logger().info('  ✅ Distance OK')
        else:
            self.get_logger().warn(f'  ❌ Distance out of range!')
        
        # Check parallel alignment
        is_parallel, parallel_error = self.check_parallel_alignment(yaw)
        self.get_logger().info(f'Parallel alignment: {parallel_error:.1f}° error (tolerance: ±{self.parallel_tol}°)')
        if is_parallel:
            self.get_logger().info('  ✅ Alignment OK')
        else:
            self.get_logger().warn(f'  ❌ Not parallel to rack!')
        
        # Check obstacle clearance
        obstacles_clear, min_obstacle_dist = self.check_obstacle_clearance()
        self.get_logger().info(f'Obstacle clearance: {min_obstacle_dist:.1f}cm (minimum: {self.obstacle_clearance}cm)')
        if obstacles_clear:
            self.get_logger().info('  ✅ No obstacles in clearance zone')
        else:
            self.get_logger().warn(f'  ❌ Obstacle too close!')
        
        # Check shelf bounds
        within_shelf = 0 <= y <= self.shelf_width
        self.get_logger().info(f'Lateral position: {y:.2f}m (shelf width: {self.shelf_width}m)')
        if within_shelf:
            self.get_logger().info('  ✅ Within shelf bounds')
        else:
            self.get_logger().warn(f'  ❌ Outside shelf bounds!')
        
        # Overall validation
        all_valid = distance_ok and is_parallel and obstacles_clear and within_shelf
        
        self.get_logger().info('=' * 60)
        if all_valid:
            self.get_logger().info('✅ POSITION VALIDATED - Ready for scanning!')
        else:
            self.get_logger().error('❌ POSITION INVALID - Adjust robot position')
        self.get_logger().info('=' * 60)
        
        # Create metadata
        from datetime import datetime
        metadata = ValidationMetadata(
            rack_id=rack_id,
            x=x,
            y=y,
            yaw=yaw,
            front_wheel_distance_cm=wheel_distance_cm,
            parallel_error_deg=parallel_error,
            shelf_lateral_position_m=y,
            obstacles_clear=obstacles_clear,
            validation_timestamp=datetime.now().isoformat(),
            notes=f"Validated during mapping phase. Obstacle distance: {min_obstacle_dist:.1f}cm"
        )
        
        return metadata if all_valid else None
    
    def save_waypoint(self, metadata: ValidationMetadata) -> bool:
        """
        Save validated waypoint to waypoints.yaml.
        
        Args:
            metadata: Validation metadata
            
        Returns:
            True if saved successfully
        """
        try:
            # Load existing waypoints
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {'waypoints': []}
            
            # Check if rack already exists
            waypoints = data.get('waypoints', [])
            existing_index = None
            for i, wp in enumerate(waypoints):
                if wp.get('rack_id') == metadata.rack_id:
                    existing_index = i
                    break
            
            # Create waypoint entry
            waypoint = {
                'id': existing_index + 1 if existing_index is not None else len(waypoints) + 1,
                'name': f'Rack_{metadata.rack_id}',
                'x': round(metadata.x, 3),
                'y': round(metadata.y, 3),
                'yaw': round(metadata.yaw, 4),
                'z': 0.0,
                'rack_id': metadata.rack_id,
                'validation': asdict(metadata)
            }
            
            # Update or append
            if existing_index is not None:
                waypoints[existing_index] = waypoint
                self.get_logger().info(f'Updated existing waypoint for {metadata.rack_id}')
            else:
                waypoints.append(waypoint)
                self.get_logger().info(f'Added new waypoint for {metadata.rack_id}')
            
            # Update total count
            data['waypoints'] = waypoints
            data['total_count'] = len(waypoints)
            
            # Save to file
            with open(self.waypoints_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            self.get_logger().info(f'Saved to: {self.waypoints_file}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoint: {e}')
            return False


def main(args=None):
    parser = argparse.ArgumentParser(description='Validate and save scanning waypoint')
    parser.add_argument('--save', type=str, help='Save waypoint for rack ID (e.g., R01)')
    parser.add_argument('--check', action='store_true', help='Check current position only')
    parsed_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    
    node = WaypointValidator()
    
    # Wait for initial data
    node.get_logger().info('Waiting for robot pose and laser scan...')
    timeout = 10.0
    start_time = node.get_clock().now()
    data_source_logged = False
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        
        # Check if we can get position (via AMCL or TF) and have scan data
        position = node.get_current_position()
        if position is not None and node.latest_scan is not None:
            # Log which data source is being used
            if not data_source_logged:
                if node.current_pose is not None:
                    node.get_logger().info('✓ Using AMCL pose for localization')
                else:
                    node.get_logger().info('✓ Using TF (SLAM) for localization')
                data_source_logged = True
            break
        
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout:
            node.get_logger().error('Timeout waiting for robot data')
            node.get_logger().error('Make sure SLAM/localization is running and robot is initialized')
            node.get_logger().error(f'Status: pose={position is not None}, scan={node.latest_scan is not None}')
            node.destroy_node()
            rclpy.shutdown()
            return
    
    # Perform validation
    if parsed_args.save:
        rack_id = parsed_args.save.upper()
        node.get_logger().info(f'\nValidating position for rack {rack_id}...\n')
        
        metadata = node.validate_current_position(rack_id)
        
        if metadata:
            node.save_waypoint(metadata)
            node.get_logger().info('\n✅ Waypoint saved successfully!')
        else:
            node.get_logger().error('\n❌ Position validation failed - waypoint NOT saved')
            node.get_logger().info('Adjust robot position and try again')
    
    elif parsed_args.check:
        node.get_logger().info('\nChecking current position...\n')
        metadata = node.validate_current_position('CHECK')
    
    else:
        node.get_logger().warn('Usage: ros2 run scuttle_navigation2 waypoint_validator.py --save R01')
        node.get_logger().warn('   or: ros2 run scuttle_navigation2 waypoint_validator.py --check')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
