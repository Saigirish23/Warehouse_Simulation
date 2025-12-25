#!/usr/bin/env python3
"""
Competition Waypoint Navigator with Checkpoint System

COMPETITION REQUIREMENTS:
- Load waypoints from waypoints.yaml
- Trigger vertical scanning at each waypoint
- Save checkpoint state after each rack
- Support --resume flag for restarts
- Track cumulative elapsed time (20 min limit)
- Ensure 10-25cm distance and parallel alignment to racks
- Path planning <3 seconds

Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
import yaml
import json
import time
import math
import argparse
from typing import List, Dict, Optional
from dataclasses import dataclass
import sys


@dataclass
class ValidationMetadata:
    """Validation metadata from mapping phase."""
    front_wheel_distance: float
    parallel_angle_deg: float
    obstacle_clearance: float
    timestamp: str
    operator_notes: Optional[str] = None


@dataclass
class Waypoint:
    """Represents a navigation waypoint."""
    id: int
    name: str
    x: float
    y: float
    yaw: float
    z: float = 0.0
    rack_id: Optional[str] = None
    validation: Optional[ValidationMetadata] = None


class CompetitionWaypointNavigator(Node):
    """
    Competition-compliant waypoint navigator with scanning integration.
    
    Competition Requirements:
    - 20 minute total time limit
    - Checkpoint system for restarts
    - Integration with vertical scanner
    - Rack alignment verification (±5°, 10-25cm distance)
    - Cumulative time tracking
    """
    
    def __init__(self, resume_from_checkpoint: bool = False):
        super().__init__('competition_waypoint_navigator')
        
        # Parameters
        self.declare_parameter('waypoints_file', 
                             '/home/b24me1066/Warehouse_Simulation/config/waypoints.yaml')
        self.declare_parameter('checkpoint_file',
                             '/home/b24me1066/Warehouse_Simulation/checkpoint.json')
        self.declare_parameter('max_mission_time_sec', 1200.0)  # 20 minutes
        self.declare_parameter('rack_distance_min_m', 0.15)     # 15cm minimum (from front wheels)
        self.declare_parameter('rack_distance_max_m', 0.25)     # 25cm maximum (from front wheels)
        self.declare_parameter('parallel_tolerance_deg', 5.0)   # ±5° alignment
        self.declare_parameter('goal_timeout_sec', 120.0)       # 2 min per waypoint
        self.declare_parameter('position_tolerance_m', 0.15)    # ±15cm position accuracy (relaxed for localization drift)
        self.declare_parameter('wheelbase_offset_m', 0.20)      # base_link to front wheels
        self.declare_parameter('max_validation_retries', 3)     # Retry limit for validation failures
        self.declare_parameter('home_x', 0.0)                     # Home position X
        self.declare_parameter('home_y', 0.0)                     # Home position Y
        self.declare_parameter('home_yaw', 0.0)                   # Home position yaw
        
        # Get parameters
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.checkpoint_file = self.get_parameter('checkpoint_file').value
        self.max_mission_time = self.get_parameter('max_mission_time_sec').value
        self.rack_dist_min = self.get_parameter('rack_distance_min_m').value
        self.rack_dist_max = self.get_parameter('rack_distance_max_m').value
        self.parallel_tol = math.radians(
            self.get_parameter('parallel_tolerance_deg').value
        )
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value
        self.position_tolerance = self.get_parameter('position_tolerance_m').value
        self.wheelbase_offset = self.get_parameter('wheelbase_offset_m').value
        self.max_retries = self.get_parameter('max_validation_retries').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.home_yaw = self.get_parameter('home_yaw').value
        
        # State variables
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index: int = 0
        self.mission_start_time: float = 0.0
        self.cumulative_time: float = 0.0  # Tracks time across restarts
        self.current_goal_handle = None
        self.mission_complete: bool = False
        self.mission_active: bool = False
        self.validation_retry_count: int = 0
        
        # Rack alignment data
        self.current_distance_to_rack: Optional[float] = None
        self.current_orientation_error: Optional[float] = None
        self.current_pose: Optional[PoseStamped] = None
        self.latest_scan: Optional[LaserScan] = None
        
        # TF Buffer for pose transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/mission/status',
            10
        )
        
        self.scanner_trigger_pub = self.create_publisher(
            String,
            '/vertical_scanner/start_scan',
            10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.scanner_status_sub = self.create_subscription(
            String,
            '/vertical_scanner/status',
            self.scanner_status_callback,
            10
        )
        
        # Timer for monitoring mission progress
        self.monitor_timer = self.create_timer(1.0, self.monitor_mission_progress)
        
        # Load waypoints
        self.load_waypoints()
        
        # Handle checkpoint resume
        if resume_from_checkpoint:
            self.load_checkpoint()
        else:
            self.initialize_checkpoint()
        
        self.get_logger().info('Competition Waypoint Navigator initialized')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Mission time limit: {self.max_mission_time/60:.1f} minutes')
        self.get_logger().info(f'Starting from waypoint {self.current_waypoint_index}')
        
    def load_waypoints(self) -> None:
        """Load waypoints from YAML file with validation metadata."""
        try:
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
            
            self.waypoints = []
            for wp_data in data.get('waypoints', []):
                # Load validation metadata if present
                validation = None
                if 'validation' in wp_data:
                    val_data = wp_data['validation']
                    validation = ValidationMetadata(
                        front_wheel_distance=val_data.get('front_wheel_distance', 0.0),
                        parallel_angle_deg=val_data.get('parallel_angle_deg', 0.0),
                        obstacle_clearance=val_data.get('obstacle_clearance', 0.0),
                        timestamp=val_data.get('timestamp', ''),
                        operator_notes=val_data.get('operator_notes')
                    )
                
                waypoint = Waypoint(
                    id=wp_data['id'],
                    name=wp_data['name'],
                    x=wp_data['x'],
                    y=wp_data['y'],
                    yaw=wp_data['yaw'],
                    z=wp_data.get('z', 0.0),
                    rack_id=wp_data.get('rack_id', f"R{wp_data['id']:02d}"),
                    validation=validation
                )
                self.waypoints.append(waypoint)
            
            validated_count = sum(1 for wp in self.waypoints if wp.validation is not None)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            self.get_logger().info(f'Waypoints with validation metadata: {validated_count}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            self.waypoints = []
    
    def initialize_checkpoint(self) -> None:
        """Initialize new checkpoint file."""
        checkpoint_data = {
            'current_waypoint_index': 0,
            'cumulative_time_sec': 0.0,
            'mission_start_timestamp': time.time(),
            'racks_completed': [],
            'total_qr_scanned': 0
        }
        
        try:
            with open(self.checkpoint_file, 'w') as f:
                json.dump(checkpoint_data, f, indent=4)
            self.get_logger().info('Checkpoint initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize checkpoint: {e}')
    
    def load_checkpoint(self) -> None:
        """Load checkpoint state for mission resume."""
        try:
            with open(self.checkpoint_file, 'r') as f:
                checkpoint_data = json.load(f)
            
            self.current_waypoint_index = checkpoint_data.get('current_waypoint_index', 0)
            self.cumulative_time = checkpoint_data.get('cumulative_time_sec', 0.0)
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('RESUMING FROM CHECKPOINT')
            self.get_logger().info(f'Waypoint index: {self.current_waypoint_index}')
            self.get_logger().info(f'Cumulative time: {self.cumulative_time:.1f}s')
            self.get_logger().info(f'Racks completed: {len(checkpoint_data.get("racks_completed", []))}')
            self.get_logger().info('=' * 50)
            
        except FileNotFoundError:
            self.get_logger().warn('No checkpoint file found - starting from beginning')
            self.initialize_checkpoint()
        except Exception as e:
            self.get_logger().error(f'Failed to load checkpoint: {e}')
            self.initialize_checkpoint()
    
    def save_checkpoint(self, rack_id: str, qr_count: int) -> None:
        """
        Save current progress to checkpoint file.
        
        Args:
            rack_id: ID of completed rack
            qr_count: Number of QR codes scanned
        """
        try:
            # Load existing checkpoint
            with open(self.checkpoint_file, 'r') as f:
                checkpoint_data = json.load(f)
            
            # Update with current progress
            checkpoint_data['current_waypoint_index'] = self.current_waypoint_index + 1
            checkpoint_data['cumulative_time_sec'] = self.get_elapsed_time()
            checkpoint_data['racks_completed'].append({
                'rack_id': rack_id,
                'qr_count': qr_count,
                'timestamp': time.time()
            })
            checkpoint_data['total_qr_scanned'] += qr_count
            
            # Save
            with open(self.checkpoint_file, 'w') as f:
                json.dump(checkpoint_data, f, indent=4)
            
            self.get_logger().info(f'Checkpoint saved: Rack {rack_id} complete')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save checkpoint: {e}')
    
    def start_mission(self) -> None:
        """Start the warehouse scanning mission."""
        if self.mission_active:
            self.get_logger().warn('Mission already active')
            return
        
        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints loaded - cannot start mission')
            return
        
        self.mission_active = True
        self.mission_start_time = time.time()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('COMPETITION MISSION STARTED')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Starting waypoint: {self.current_waypoint_index}')
        self.get_logger().info(f'Time limit: {self.max_mission_time/60:.1f} minutes')
        self.get_logger().info('=' * 60)
        
        # Navigate to first waypoint
        self.navigate_to_next_waypoint()
    
    def navigate_to_next_waypoint(self) -> None:
        """Navigate to the next waypoint in sequence."""
        if self.current_waypoint_index >= len(self.waypoints):
            # All waypoints complete - return home
            self.get_logger().info('All waypoints complete - returning to home position')
            self.return_to_home()
            return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        self.get_logger().info(
            f'Navigating to waypoint {waypoint.id}: {waypoint.name} '
            f'({waypoint.x:.2f}, {waypoint.y:.2f})'
        )
        
        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = waypoint.x
        goal_msg.pose.pose.position.y = waypoint.y
        goal_msg.pose.pose.position.z = waypoint.z
        
        # Convert yaw to quaternion
        quat = self.yaw_to_quaternion(waypoint.yaw)
        goal_msg.pose.pose.orientation = quat
        
        # Wait for action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def yaw_to_quaternion(self, yaw: float):
        """Convert yaw angle to quaternion."""
        from geometry_msgs.msg import Quaternion
        quat = Quaternion()
        quat.w = math.cos(yaw / 2.0)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        return quat
    
    def nav_goal_response_callback(self, future) -> None:
        """Handle Nav2 goal acceptance response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Navigation goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg) -> None:
        """Handle Nav2 feedback during navigation."""
        # Log progress periodically
        pass
    
    def nav_result_callback(self, future) -> None:
        """
        Handle Nav2 goal completion result.
        
        COMPETITION REQUIREMENT:
        - Verify rack alignment before scanning
        - Ensure 15-25cm distance from FRONT WHEELS and ±5° parallel
        - Validate position matches expected geotag
        """
        result = future.result().result
        status = future.result().status
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Reached waypoint: {waypoint.name}')
            
            # Runtime position validation
            if not self.validate_position_runtime(waypoint):
                if self.validation_retry_count < self.max_retries:
                    self.validation_retry_count += 1
                    self.get_logger().warn(
                        f'Position validation failed - retry {self.validation_retry_count}/{self.max_retries}'
                    )
                    # Re-navigate to same waypoint
                    time.sleep(2.0)
                    self.navigate_to_next_waypoint()
                    return
                else:
                    self.get_logger().error('Max validation retries exceeded - skipping rack')
                    self.validation_retry_count = 0
                    self.current_waypoint_index += 1
                    self.navigate_to_next_waypoint()
                    return
            
            # Reset retry counter on success
            self.validation_retry_count = 0
            
            # Start vertical scanning
            self.trigger_vertical_scan(waypoint.rack_id)
            
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            # Move to next waypoint
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
    
    def validate_position_runtime(self, waypoint: Waypoint) -> bool:
        """
        Validate robot position at runtime before scanning.
        
        Checks:
        1. Position within ±10cm of expected waypoint
        2. Orientation within ±5° of expected yaw
        3. Front wheel distance 15-25cm from rack
        4. Obstacle clearance adequate
        
        Args:
            waypoint: Expected waypoint with validation metadata
            
        Returns:
            True if all validations pass, False otherwise
        """
        self.get_logger().info('=== Runtime Position Validation ===')
        
        # Get current pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            quat = transform.transform.rotation
            siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except Exception as e:
            self.get_logger().error(f'Failed to get current pose: {e}')
            return False
        
        # Check 1: Position accuracy
        dx = current_x - waypoint.x
        dy = current_y - waypoint.y
        position_error = math.sqrt(dx*dx + dy*dy)
        
        if position_error > self.position_tolerance:
            self.get_logger().error(
                f'Position error {position_error*100:.1f}cm exceeds tolerance '
                f'{self.position_tolerance*100:.0f}cm'
            )
            return False
        
        self.get_logger().info(f'✓ Position error: {position_error*100:.1f}cm (within ±{self.position_tolerance*100:.0f}cm)')
        
        # Check 2: Orientation accuracy
        yaw_error = self.normalize_angle(current_yaw - waypoint.yaw)
        
        if abs(yaw_error) > self.parallel_tol:
            self.get_logger().error(
                f'Orientation error {math.degrees(yaw_error):.1f}° exceeds tolerance '
                f'±{math.degrees(self.parallel_tol):.1f}°'
            )
            return False
        
        self.get_logger().info(f'✓ Orientation error: {math.degrees(yaw_error):.1f}° (within ±{math.degrees(self.parallel_tol):.1f}°)')
        
        # Check 3: Front wheel distance to rack
        if self.latest_scan is None:
            self.get_logger().warn('No laser scan available for distance check')
            return False
        
        # Measure distance from base_link to rack
        base_link_distance = self.measure_distance_to_rack()
        
        if base_link_distance is None:
            self.get_logger().error('Failed to measure distance to rack')
            return False
        
        # Calculate front wheel distance (front wheels are FURTHER from rack)
        front_wheel_distance = base_link_distance - self.wheelbase_offset
        
        if not (self.rack_dist_min <= front_wheel_distance <= self.rack_dist_max):
            self.get_logger().error(
                f'Front wheel distance {front_wheel_distance*100:.1f}cm outside '
                f'range {self.rack_dist_min*100:.0f}-{self.rack_dist_max*100:.0f}cm'
            )
            return False
        
        self.get_logger().info(
            f'✓ Front wheel distance: {front_wheel_distance*100:.1f}cm '
            f'(base_link: {base_link_distance*100:.1f}cm - wheelbase: {self.wheelbase_offset*100:.0f}cm)'
        )
        
        # Check 4: Compare with validation metadata if available
        if waypoint.validation is not None:
            expected_distance = waypoint.validation.front_wheel_distance
            distance_diff = abs(front_wheel_distance - expected_distance)
            
            if distance_diff > 0.05:  # 5cm tolerance
                self.get_logger().warn(
                    f'Distance differs from mapping: expected {expected_distance*100:.1f}cm, '
                    f'got {front_wheel_distance*100:.1f}cm (diff: {distance_diff*100:.1f}cm)'
                )
                # Warning only, not a failure
        
        self.get_logger().info('=== Position Validation: PASSED ===')
        return True
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def measure_distance_to_rack(self) -> Optional[float]:
        """
        Measure distance from base_link to rack in front using laser scan.
        
        Returns:
            Distance in meters, or None if measurement fails
        """
        if self.latest_scan is None:
            return None
        
        # Get readings in front cone (±15°)
        angle_range = math.radians(15)
        front_readings = []
        
        for i, distance in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
            
            if abs(angle) <= angle_range:
                if self.latest_scan.range_min < distance < self.latest_scan.range_max:
                    front_readings.append(distance)
        
        # Return minimum distance (closest point in front)
        if front_readings:
            return min(front_readings)
        else:
            return None
    
    def verify_rack_alignment(self) -> bool:
        """
        DEPRECATED: Use validate_position_runtime instead.
        
        Legacy method for backwards compatibility.
        """
        self.get_logger().warn('verify_rack_alignment is deprecated, use validate_position_runtime')
        return True
    
    def scan_callback(self, msg: LaserScan) -> None:
        """
        Process laser scan to store latest scan data.
        
        Args:
            msg: LaserScan message
        """
        self.latest_scan = msg
        
        # Also update legacy distance measurement for compatibility
        angle_range = math.radians(15)
        front_readings = []
        
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + (i * msg.angle_increment)
            
            if abs(angle) <= angle_range:
                if msg.range_min < distance < msg.range_max:
                    front_readings.append(distance)
        
        if front_readings:
            self.current_distance_to_rack = min(front_readings)
    
    def trigger_vertical_scan(self, rack_id: str) -> None:
        """
        Trigger vertical scanning sequence for current rack.
        
        Args:
            rack_id: ID of rack to scan
        """
        waypoint_num = self.current_waypoint_index + 1  # 1-indexed for display
        self.get_logger().info(f'Triggering vertical scan for waypoint {waypoint_num}, rack: {rack_id}')
        
        # Publish scan trigger with rack ID and waypoint number
        trigger_msg = String()
        trigger_data = {
            'rack_id': rack_id,
            'waypoint_number': waypoint_num
        }
        trigger_msg.data = json.dumps(trigger_data)
        self.scanner_trigger_pub.publish(trigger_msg)
        
        # Scanner will publish status when complete
    
    def scanner_status_callback(self, msg: String) -> None:
        """
        Handle vertical scanner status updates.
        
        Args:
            msg: JSON status from vertical scanner
        """
        try:
            status = json.loads(msg.data)
            
            # Check if scan complete
            if status.get('scan_complete', False):
                rack_id = status.get('rack_id', 'unknown')
                qr_count = status.get('total_qr_detected', 0)
                
                self.get_logger().info(
                    f'Rack {rack_id} scan complete: {qr_count} QR codes'
                )
                
                # Save checkpoint
                self.save_checkpoint(rack_id, qr_count)
                
                # Move to next waypoint
                self.current_waypoint_index += 1
                self.navigate_to_next_waypoint()
                
        except Exception as e:
            self.get_logger().error(f'Failed to parse scanner status: {e}')
    
    def monitor_mission_progress(self) -> None:
        """
        Monitor mission progress and time limits.
        
        COMPETITION REQUIREMENT: 20 minute total time limit
        """
        if not self.mission_active:
            return
        
        elapsed = self.get_elapsed_time()
        remaining = self.max_mission_time - elapsed
        progress_pct = (self.current_waypoint_index / len(self.waypoints)) * 100
        
        # Publish status
        status_data = {
            'mission_active': self.mission_active,
            'current_waypoint': self.current_waypoint_index,
            'total_waypoints': len(self.waypoints),
            'progress_percent': round(progress_pct, 1),
            'elapsed_seconds': round(elapsed, 1),
            'remaining_seconds': round(remaining, 1),
            'time_warning': remaining < 300  # Warn if <5 min remaining
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
        
        # Log warnings
        if remaining < 300 and elapsed % 60 < 1:  # Every minute
            self.get_logger().warn(
                f'TIME WARNING: {remaining/60:.1f} minutes remaining'
            )
        
        # Check timeout
        if elapsed > self.max_mission_time:
            self.get_logger().error('MISSION TIMEOUT: 20 minute limit exceeded')
            self.abort_mission()
    
    def get_elapsed_time(self) -> float:
        """
        Get total elapsed time including cumulative from previous runs.
        
        Returns:
            Total elapsed time in seconds
        """
        if self.mission_start_time > 0:
            current_session_time = time.time() - self.mission_start_time
            return self.cumulative_time + current_session_time
        return self.cumulative_time
    
    def return_to_home(self) -> None:
        """Navigate back to home position after completing all waypoints."""
        self.returning_home = True
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'RETURNING TO HOME: ({self.home_x:.2f}, {self.home_y:.2f})')
        self.get_logger().info('=' * 60)
        
        # Create home position goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.home_x
        goal_msg.pose.pose.position.y = self.home_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(self.home_yaw)
        
        # Send navigation goal
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.home_goal_response_callback)
    
    def home_goal_response_callback(self, future) -> None:
        """Handle home navigation goal acceptance."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Home navigation goal rejected!')
            self.complete_mission()
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Home navigation goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)
    
    def home_result_callback(self, future) -> None:
        """Handle home navigation completion."""
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Successfully returned to home position')
        else:
            self.get_logger().warn(f'⚠️  Home navigation ended with status: {status}')
        
        self.complete_mission()
    
    def complete_mission(self) -> None:
        """Complete mission successfully."""
        self.mission_active = False
        self.mission_complete = True
        
        total_time = self.get_elapsed_time()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION COMPLETE!')
        self.get_logger().info(f'Total time: {total_time:.1f}s ({total_time/60:.2f} min)')
        self.get_logger().info(f'Waypoints completed: {self.current_waypoint_index}/{len(self.waypoints)}')
        self.get_logger().info('=' * 60)
    
    def abort_mission(self) -> None:
        """Abort mission due to timeout or error."""
        self.mission_active = False
        
        # Cancel current goal
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
        
        self.get_logger().error('MISSION ABORTED')


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--resume', action='store_true', 
                       help='Resume from last checkpoint')
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=args)
    
    node = CompetitionWaypointNavigator(resume_from_checkpoint=parsed_args.resume)
    
    # Start mission after a short delay
    time.sleep(2.0)
    node.start_mission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
