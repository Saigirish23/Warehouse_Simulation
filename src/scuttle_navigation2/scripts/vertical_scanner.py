#!/usr/bin/env python3
"""
Vertical Scanner for Warehouse Rack QR Code Detection

COMPETITION REQUIREMENTS:
- Scan QR codes from 20cm to 180cm height (Z-axis)
- Complete each rack in <3 minutes (180 seconds)
- Vertical positioning accuracy: ±2cm
- Return to home position after scan

This node controls a linear slider to move the camera vertically
and triggers QR detection at each height level.

Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Image
import time
import json
from typing import Optional, List, Dict
from dataclasses import dataclass
import cv2
from cv_bridge import CvBridge
import subprocess


@dataclass
class ScanLevel:
    """Represents a single height level to scan."""
    height_cm: float
    scanned: bool = False
    qr_count: int = 0
    timestamp: Optional[float] = None


class VerticalScanner(Node):
    """
    Control Z-axis slider for multi-level QR code scanning.
    
    Competition Requirements:
    - Height range: 20cm to 180cm
    - Positioning accuracy: ±2cm
    - Complete scan: <180 seconds
    - Integration with qr_scan module
    """
    
    def __init__(self):
        super().__init__('vertical_scanner')
        
        # Parameters
        self.declare_parameter('min_height_cm', 20.0)       # 20cm minimum
        self.declare_parameter('max_height_cm', 150.0)      # 150cm maximum
        self.declare_parameter('height_increment_cm', 20.0) # 20cm steps
        self.declare_parameter('positioning_tolerance_cm', 2.0)  # ±2cm accuracy
        self.declare_parameter('settle_time_sec', 1.0)      # Time to stabilize after move
        self.declare_parameter('max_scan_duration_sec', 180.0)  # 3 minute limit
        self.declare_parameter('home_position_cm', 20.0)    # Default home
        self.declare_parameter('current_rack_id', 'R01')    # Current rack being scanned
        self.declare_parameter('display_camera_feed', True)  # Show live camera feed
        
        # Get parameters
        self.min_height = self.get_parameter('min_height_cm').value
        self.max_height = self.get_parameter('max_height_cm').value
        self.height_increment = self.get_parameter('height_increment_cm').value
        self.tolerance = self.get_parameter('positioning_tolerance_cm').value
        self.settle_time = self.get_parameter('settle_time_sec').value
        self.max_duration = self.get_parameter('max_scan_duration_sec').value
        self.home_position = self.get_parameter('home_position_cm').value
        self.current_rack_id = self.get_parameter('current_rack_id').value
        self.display_camera = self.get_parameter('display_camera_feed').value
        
        # State variables
        self.current_height_cm: float = self.home_position
        self.target_height_cm: Optional[float] = None
        self.scan_in_progress: bool = False
        self.scan_start_time: Optional[float] = None
        self.scan_levels: List[ScanLevel] = []
        self.total_qr_detected: int = 0
        self.scan_direction: str = 'up'  # 'up' or 'down'
        self.waypoint_number: int = 1  # Current waypoint number
        
        # CV Bridge for image processing
        self.bridge = CvBridge()
        
        # Publishers
        self.height_command_pub = self.create_publisher(
            Float32,
            '/slider/height_command',
            10
        )
        
        self.scan_trigger_pub = self.create_publisher(
            Bool,
            '/qr_scanner/trigger',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/vertical_scanner/status',
            10
        )
        
        # Subscribers
        self.height_feedback_sub = self.create_subscription(
            Float32,
            '/slider/height_feedback',
            self.height_feedback_callback,
            10
        )
        
        self.scan_complete_sub = self.create_subscription(
            String,
            '/qr_scanner/result',
            self.scan_result_callback,
            10
        )
        
        self.scan_command_sub = self.create_subscription(
            String,
            '/vertical_scanner/start_scan',
            self.start_scan_callback,
            10
        )
        
        # Camera image subscriber for live feed
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        # Timer for monitoring scan progress
        self.monitor_timer = self.create_timer(0.5, self.monitor_scan_progress)
        
        # Initialize scan levels
        self.initialize_scan_levels()
        
        self.get_logger().info('Vertical Scanner initialized')
        self.get_logger().info(f'Height range: {self.min_height}cm - {self.max_height}cm')
        self.get_logger().info(f'Scan levels: {len(self.scan_levels)} positions')
        self.get_logger().info(f'Max scan duration: {self.max_duration}s')
    
    def initialize_scan_levels(self) -> None:
        """Initialize list of height levels to scan."""
        self.scan_levels = []
        
        height = self.min_height
        while height <= self.max_height:
            self.scan_levels.append(ScanLevel(height_cm=height))
            height += self.height_increment
        
        self.get_logger().info(f'Initialized {len(self.scan_levels)} scan levels')
    
    def camera_callback(self, msg: Image) -> None:
        """Display live camera feed during scanning."""
        if not self.scan_in_progress or not self.display_camera:
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add overlay information
            height_text = f'Height: {self.current_height_cm:.1f}cm'
            direction_text = f'Direction: {self.scan_direction.upper()}'
            waypoint_text = f'Waypoint: {self.waypoint_number}'
            rack_text = f'Rack: {self.current_rack_id}'
            
            # Draw text overlay
            cv2.putText(cv_image, height_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, direction_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, waypoint_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(cv_image, rack_text, (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Display the image
            cv2.imshow('Vertical Scanner - Live Feed', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Failed to display camera feed: {e}')
    
    def start_scan_callback(self, msg: String) -> None:
        """
        Start vertical scanning sequence.
        
        Args:
            msg: JSON with rack_id and waypoint_number parameters
        """
        if self.scan_in_progress:
            self.get_logger().warn('Scan already in progress')
            return
        
        try:
            data = json.loads(msg.data)
            self.current_rack_id = data.get('rack_id', self.current_rack_id)
            self.waypoint_number = data.get('waypoint_number', 1)
            
            # Determine scan direction based on waypoint number
            # Odd waypoints (1,3,5,7,9): scan UP (bottom to top)
            # Even waypoints (2,4,6,8,10): scan DOWN (top to bottom)
            self.scan_direction = 'up' if self.waypoint_number % 2 == 1 else 'down'
            
            self.get_logger().info(
                f'Waypoint {self.waypoint_number}: Scanning {self.scan_direction.upper()} '
                f'({"bottom→top" if self.scan_direction == "up" else "top→bottom"})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to parse scan command: {e}')
            self.scan_direction = 'up'  # Default to upward
        
        self.get_logger().info(f'Starting vertical scan for rack {self.current_rack_id}')
        self.start_scan()
    
    def start_scan(self) -> None:
        """
        Begin vertical scanning sequence.
        
        COMPETITION REQUIREMENT:
        - Complete all levels in <180 seconds
        - Position accuracy ±2cm
        - Scan from 20cm to 180cm
        """
        # Reset state
        self.scan_in_progress = True
        self.scan_start_time = time.time()
        self.total_qr_detected = 0
        
        # Reset scan levels
        for level in self.scan_levels:
            level.scanned = False
            level.qr_count = 0
            level.timestamp = None
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'VERTICAL SCAN STARTED - Rack: {self.current_rack_id}')
        self.get_logger().info(f'Waypoint: {self.waypoint_number} - Direction: {self.scan_direction.upper()}')
        self.get_logger().info(f'Levels to scan: {len(self.scan_levels)}')
        self.get_logger().info('=' * 50)
        
        # Open camera feed window if enabled
        if self.display_camera:
            cv2.namedWindow('Vertical Scanner - Live Feed', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Vertical Scanner - Live Feed', 640, 480)
        
        # Start at appropriate level based on direction
        self.scan_next_level()
    
    def scan_next_level(self) -> None:
        """Move to next unscanned level and trigger QR detection."""
        # Find next unscanned level based on scan direction
        next_level = None
        
        if self.scan_direction == 'up':
            # Scan upward: find lowest unscanned level
            for level in self.scan_levels:
                if not level.scanned:
                    next_level = level
                    break
        else:  # scan_direction == 'down'
            # Scan downward: find highest unscanned level
            for level in reversed(self.scan_levels):
                if not level.scanned:
                    next_level = level
                    break
        
        if next_level is None:
            # All levels scanned - finish
            self.get_logger().info(f'All levels scanned in {self.scan_direction} direction')
            self.finish_scan()
            return
        
        # Check time limit
        if self.scan_start_time:
            elapsed = time.time() - self.scan_start_time
            if elapsed > self.max_duration:
                self.get_logger().error(
                    f'TIMEOUT: Scan exceeded {self.max_duration}s limit'
                )
                self.finish_scan()
                return
        
        # Move to next height
        self.get_logger().info(f'Moving to height: {next_level.height_cm}cm')
        self.move_to_height(next_level.height_cm)
    
    def move_to_height(self, target_height_cm: float) -> None:
        """
        Command slider to move to target height.
        
        Args:
            target_height_cm: Target height in centimeters
        """
        self.target_height_cm = target_height_cm
        
        # Publish height command
        cmd_msg = Float32()
        cmd_msg.data = target_height_cm
        self.height_command_pub.publish(cmd_msg)
        
        self.get_logger().info(f'Height command sent: {target_height_cm}cm')
    
    def height_feedback_callback(self, msg: Float32) -> None:
        """
        Receive current height feedback from slider.
        
        Args:
            msg: Current height in cm
        """
        self.current_height_cm = msg.data
        
        # Check if we've reached target
        if self.target_height_cm is not None:
            error = abs(self.current_height_cm - self.target_height_cm)
            
            if error <= self.tolerance:
                # Position reached with required accuracy
                self.get_logger().info(
                    f'Position reached: {self.current_height_cm:.1f}cm '
                    f'(error: {error:.2f}cm)'
                )
                
                # Wait for settling
                time.sleep(self.settle_time)
                
                # Trigger QR scan at this height
                self.trigger_qr_scan()
                
                # Clear target
                self.target_height_cm = None
    
    def trigger_qr_scan(self) -> None:
        """
        Trigger QR code detection at current height.
        
        COMPETITION REQUIREMENT:
        - 1920×1080 resolution
        - Detection rate >90%
        - Parse RACKID_SHELFID_ITEMCODE format
        """
        self.get_logger().info(f'Triggering QR scan at {self.current_height_cm}cm')
        
        # Publish trigger with metadata
        trigger_msg = Bool()
        trigger_msg.data = True
        self.scan_trigger_pub.publish(trigger_msg)
        
        # Note: Waiting for scan result callback
    
    def scan_result_callback(self, msg: String) -> None:
        """
        Receive QR scan results from qr_scanner node.
        
        Args:
            msg: JSON with scan results
        """
        try:
            result = json.loads(msg.data)
            qr_count = result.get('qr_count', 0)
            success = result.get('success', False)
            height = result.get('height_cm', self.current_height_cm)
            
            # Find matching scan level
            for level in self.scan_levels:
                if abs(level.height_cm - height) < self.tolerance:
                    level.scanned = True
                    level.qr_count = qr_count
                    level.timestamp = time.time()
                    self.total_qr_detected += qr_count
                    
                    self.get_logger().info(
                        f'Level {height}cm complete: {qr_count} QR codes detected'
                    )
                    break
            
            # Move to next level
            self.scan_next_level()
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse scan result: {e}')
    
    def finish_scan(self) -> None:
        """
        Complete scanning sequence and return to home.
        
        COMPETITION REQUIREMENT:
        - Return to home position
        - Generate scan report
        - Log completion time
        """
        if not self.scan_in_progress:
            return
        
        self.scan_in_progress = False
        
        # Calculate statistics
        if self.scan_start_time:
            total_time = time.time() - self.scan_start_time
            scanned_count = sum(1 for level in self.scan_levels if level.scanned)
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('VERTICAL SCAN COMPLETE')
            self.get_logger().info(f'Rack ID: {self.current_rack_id}')
            self.get_logger().info(f'Levels scanned: {scanned_count}/{len(self.scan_levels)}')
            self.get_logger().info(f'Total QR detected: {self.total_qr_detected}')
            self.get_logger().info(f'Duration: {total_time:.1f}s / {self.max_duration}s')
            self.get_logger().info('=' * 50)
            
            # Check competition requirement
            if total_time > self.max_duration:
                self.get_logger().error(
                    f'COMPETITION VIOLATION: Scan time {total_time:.1f}s '
                    f'exceeds {self.max_duration}s limit'
                )
        
        # Return to home position
        self.get_logger().info(f'Returning to home position: {self.home_position}cm')
        self.move_to_height(self.home_position)
        
        # Close camera feed window
        if self.display_camera:
            cv2.destroyWindow('Vertical Scanner - Live Feed')
        
        # Publish completion status
        self.publish_completion_status()
    
    def monitor_scan_progress(self) -> None:
        """Monitor and publish scan progress."""
        if not self.scan_in_progress or not self.scan_start_time:
            return
        
        elapsed = time.time() - self.scan_start_time
        scanned = sum(1 for level in self.scan_levels if level.scanned)
        progress_pct = (scanned / len(self.scan_levels)) * 100
        
        # Publish status
        status_data = {
            'rack_id': self.current_rack_id,
            'scan_in_progress': self.scan_in_progress,
            'current_height_cm': round(self.current_height_cm, 1),
            'levels_scanned': scanned,
            'total_levels': len(self.scan_levels),
            'progress_percent': round(progress_pct, 1),
            'elapsed_seconds': round(elapsed, 1),
            'time_remaining_seconds': round(self.max_duration - elapsed, 1),
            'total_qr_detected': self.total_qr_detected
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
    def publish_completion_status(self) -> None:
        """Publish final scan completion status."""
        status_data = {
            'rack_id': self.current_rack_id,
            'scan_in_progress': False,
            'scan_complete': True,
            'total_qr_detected': self.total_qr_detected,
            'levels_scanned': sum(1 for level in self.scan_levels if level.scanned),
            'total_levels': len(self.scan_levels)
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = VerticalScanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
