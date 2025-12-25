#!/usr/bin/env python3
"""
Competition-Compliant QR Code Scanner

COMPETITION REQUIREMENTS:
- 1920×1080 minimum resolution
- QR format: "RACKID_SHELFID_ITEMCODE" (e.g., R03_S2_ITM430)
- Detection rate >90% for 5cm×5cm codes
- Associate with rack ID and height
- Prevent duplicates per rack
- Sharp focus and image enhancement

Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import json
import os
from datetime import datetime
from typing import Optional, List, Dict, Tuple
import re


class QRCodeScanner(Node):
    """
    High-resolution QR code scanner with competition-compliant features.
    
    Competition Requirements:
    - 1920×1080 resolution
    - >90% detection rate
    - Parse RACKID_SHELFID_ITEMCODE format
    - Associate with height and rack ID
    - No duplicates per rack
    """
    
    def __init__(self):
        super().__init__('qr_code_scanner')
        
        # Parameters
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('resolution_width', 1920)
        self.declare_parameter('resolution_height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('scan_duration_sec', 3.0)     # Time per height level
        self.declare_parameter('json_file', '/home/b24me1066/Warehouse_Simulation/scanned_qrs.json')
        self.declare_parameter('enable_display', False)      # Show camera feed
        self.declare_parameter('current_rack_id', 'R01')
        self.declare_parameter('current_height_cm', 20.0)
        
        # Get parameters
        self.camera_device = self.get_parameter('camera_device').value
        self.res_width = self.get_parameter('resolution_width').value
        self.res_height = self.get_parameter('resolution_height').value
        self.fps = self.get_parameter('fps').value
        self.scan_duration = self.get_parameter('scan_duration_sec').value
        self.json_file = self.get_parameter('json_file').value
        self.enable_display = self.get_parameter('enable_display').value
        self.current_rack_id = self.get_parameter('current_rack_id').value
        self.current_height_cm = self.get_parameter('current_height_cm').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera
        self.cap: Optional[cv2.VideoCapture] = None
        self.initialize_camera()
        
        # State
        self.scanning_active: bool = False
        self.scan_start_time: Optional[float] = None
        self.detected_qrs: Dict[str, Dict] = {}  # Prevent duplicates in current scan
        
        # Initialize JSON database
        self.init_json_database()
        
        # Subscribers
        self.trigger_sub = self.create_subscription(
            Bool,
            '/qr_scanner/trigger',
            self.trigger_callback,
            10
        )
        
        self.height_sub = self.create_subscription(
            Float32,
            '/slider/height_feedback',
            self.height_callback,
            10
        )
        
        self.rack_id_sub = self.create_subscription(
            String,
            '/qr_scanner/set_rack_id',
            self.rack_id_callback,
            10
        )
        
        # Publishers
        self.result_pub = self.create_publisher(
            String,
            '/qr_scanner/result',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/qr_scanner/status',
            10
        )
        
        # Timer for scanning loop
        self.scan_timer = self.create_timer(0.033, self.scan_loop)  # ~30 Hz
        
        self.get_logger().info('QR Code Scanner initialized')
        self.get_logger().info(f'Resolution: {self.res_width}x{self.res_height}')
        self.get_logger().info(f'JSON database: {self.json_file}')
    
    def initialize_camera(self) -> None:
        """
        Initialize camera with competition-compliant resolution.
        
        COMPETITION REQUIREMENT: 1920×1080 minimum resolution
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_device)
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera device {self.camera_device}')
                return
            
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.res_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.res_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Enable autofocus if available
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
            
            # Verify actual resolution
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            
            self.get_logger().info(
                f'Camera initialized: {int(actual_width)}x{int(actual_height)}'
            )
            
            # Competition requirement check
            if actual_width < 1920 or actual_height < 1080:
                self.get_logger().warn(
                    f'COMPETITION WARNING: Resolution {int(actual_width)}x{int(actual_height)} '
                    f'is below required 1920×1080'
                )
            
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {e}')
    
    def init_json_database(self) -> None:
        """Initialize JSON database file if it doesn't exist."""
        if not os.path.exists(self.json_file):
            with open(self.json_file, 'w') as f:
                json.dump({"qr_codes": []}, f, indent=4)
            self.get_logger().info(f'Created JSON database: {self.json_file}')
    
    def trigger_callback(self, msg: Bool) -> None:
        """
        Start/stop QR scanning on trigger.
        
        Args:
            msg: True to start scanning, False to stop
        """
        if msg.data and not self.scanning_active:
            self.start_scan()
        elif not msg.data and self.scanning_active:
            self.stop_scan()
    
    def height_callback(self, msg: Float32) -> None:
        """Update current height for QR metadata."""
        self.current_height_cm = msg.data
    
    def rack_id_callback(self, msg: String) -> None:
        """Update current rack ID for QR metadata."""
        self.current_rack_id = msg.data
        self.get_logger().info(f'Rack ID updated: {self.current_rack_id}')
    
    def start_scan(self) -> None:
        """Start QR code scanning sequence."""
        self.scanning_active = True
        self.scan_start_time = self.get_clock().now().nanoseconds / 1e9
        self.detected_qrs = {}
        
        self.get_logger().info(
            f'QR scan started: Rack {self.current_rack_id}, '
            f'Height {self.current_height_cm}cm'
        )
    
    def stop_scan(self) -> None:
        """Stop scanning and publish results."""
        self.scanning_active = False
        
        # Publish results
        self.publish_scan_results()
    
    def scan_loop(self) -> None:
        """Main scanning loop - runs continuously."""
        if not self.scanning_active or self.cap is None:
            return
        
        # Check scan timeout
        if self.scan_start_time:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.scan_start_time
            if elapsed > self.scan_duration:
                self.get_logger().info('Scan duration complete')
                self.stop_scan()
                return
        
        # Capture frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame', throttle_duration_sec=1.0)
            return
        
        # Enhance image for better QR detection
        enhanced_frame = self.enhance_image(frame)
        
        # Decode QR codes
        qr_codes = decode(enhanced_frame)
        
        # Process detected codes
        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            
            # Validate format and save
            if self.validate_and_save_qr(qr_data, frame, qr):
                # Draw bounding box on frame
                points = qr.polygon
                if len(points) == 4:
                    pts = np.array(points, dtype=np.int32)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                    cv2.putText(
                        frame, qr_data, 
                        (points[0].x, points[0].y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                    )
        
        # Display frame if enabled
        if self.enable_display:
            cv2.imshow('QR Scanner', frame)
            cv2.waitKey(1)
    
    def enhance_image(self, frame: np.ndarray) -> np.ndarray:
        """
        Enhance image for better QR code detection.
        
        Args:
            frame: Input image
            
        Returns:
            Enhanced image
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply sharpening
        blur = cv2.GaussianBlur(gray, (0, 0), 3)
        sharp = cv2.addWeighted(gray, 1.8, blur, -0.8, 0)
        
        # Adaptive thresholding for varying lighting
        enhanced = cv2.adaptiveThreshold(
            sharp, 255, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 
            11, 2
        )
        
        return enhanced
    
    def validate_and_save_qr(
        self, 
        qr_data: str, 
        frame: np.ndarray,
        qr_obj
    ) -> bool:
        """
        Validate QR format and save to database.
        
        COMPETITION REQUIREMENT:
        - Format: RACKID_SHELFID_ITEMCODE (e.g., R03_S2_ITM430)
        - No duplicates per rack
        
        Args:
            qr_data: QR code string
            frame: Current camera frame
            qr_obj: Decoded QR object
            
        Returns:
            True if valid and saved, False otherwise
        """
        # Check if already detected in this scan
        if qr_data in self.detected_qrs:
            return False
        
        # Validate format: RACKID_SHELFID_ITEMCODE
        pattern = r'^R\d{2}_S\d+_ITM\d+$'
        if not re.match(pattern, qr_data):
            self.get_logger().warn(
                f'Invalid QR format: {qr_data} (expected: R##_S#_ITM###)',
                throttle_duration_sec=1.0
            )
            return False
        
        # Parse components
        parts = qr_data.split('_')
        rack_id = parts[0]
        shelf_id = parts[1]
        item_code = parts[2]
        
        # Check if this QR belongs to current rack
        if rack_id != self.current_rack_id:
            self.get_logger().warn(
                f'QR {qr_data} belongs to {rack_id}, scanning {self.current_rack_id}',
                throttle_duration_sec=1.0
            )
            # Still save but mark as misplaced
        
        # Save to database
        self.save_to_json({
            'rack_id': rack_id,
            'shelf_id': shelf_id,
            'item_code': item_code,
            'full_code': qr_data,
            'height_cm': round(self.current_height_cm, 1),
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'expected_rack': self.current_rack_id
        })
        
        # Mark as detected in this scan
        self.detected_qrs[qr_data] = {
            'rack_id': rack_id,
            'shelf_id': shelf_id,
            'item_code': item_code,
            'height_cm': self.current_height_cm
        }
        
        self.get_logger().info(f'QR detected: {qr_data} at {self.current_height_cm}cm')
        
        return True
    
    def save_to_json(self, qr_entry: Dict) -> None:
        """
        Save QR code to JSON database with duplicate prevention.
        
        Args:
            qr_entry: QR data dictionary
        """
        try:
            # Load existing data
            with open(self.json_file, 'r') as f:
                data = json.load(f)
            
            # Check for duplicates (same rack + code)
            for item in data['qr_codes']:
                if (item.get('full_code') == qr_entry['full_code'] and
                    item.get('expected_rack') == qr_entry['expected_rack']):
                    # Duplicate found - don't save
                    return
            
            # Add serial number
            qr_entry['sno'] = len(data['qr_codes']) + 1
            
            # Append new entry
            data['qr_codes'].append(qr_entry)
            
            # Save back to file
            with open(self.json_file, 'w') as f:
                json.dump(data, f, indent=4)
            
        except Exception as e:
            self.get_logger().error(f'Failed to save QR to JSON: {e}')
    
    def publish_scan_results(self) -> None:
        """Publish scan results after completion."""
        result_data = {
            'success': True,
            'rack_id': self.current_rack_id,
            'height_cm': self.current_height_cm,
            'qr_count': len(self.detected_qrs),
            'qr_codes': list(self.detected_qrs.keys())
        }
        
        result_msg = String()
        result_msg.data = json.dumps(result_data)
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(
            f'Scan complete: {len(self.detected_qrs)} QR codes at '
            f'{self.current_height_cm}cm'
        )
    
    def cleanup(self) -> None:
        """Clean up resources."""
        if self.cap is not None:
            self.cap.release()
        if self.enable_display:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = QRCodeScanner()
    
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
