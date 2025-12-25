#!/usr/bin/env python3
"""
Interactive Waypoint Recorder

Records and validates waypoints on spacebar press.
Automatically saves validated waypoints to YAML file.

Usage:
    ros2 run scuttle_navigation2 interactive_waypoint_recorder.py

Controls:
    SPACEBAR - Validate and save current position as waypoint
    q - Quit and finalize waypoint file
    
Author: Competition Team
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import yaml
import math
import sys
import select
import termios
import tty
import os
from typing import Optional, Tuple, List
from dataclasses import dataclass, asdict
from datetime import datetime


@dataclass
class ValidationMetadata:
    """Validation metadata for a waypoint."""
    front_wheel_distance: float
    base_link_distance: float
    parallel_angle_deg: float
    obstacle_clearance: float
    timestamp: str
    operator_notes: str = ""


@dataclass
class Waypoint:
    """Waypoint with validation metadata."""
    id: int
    name: str
    rack_id: str
    x: float
    y: float
    z: float
    yaw: float
    validation: dict


class InteractiveWaypointRecorder(Node):
    """
    Interactive waypoint recorder with real-time validation.
    
    Press SPACEBAR to record and validate current position.
    """
    
    def __init__(self, max_waypoints: int = 10):
        super().__init__('interactive_waypoint_recorder')
        
        # Parameters
        self.max_waypoints = max_waypoints
        self.wheelbase_offset = 0.20  # meters
        self.min_wheel_dist = 0.15  # 15cm
        self.max_wheel_dist = 0.25  # 25cm
        self.parallel_tol = 5.0  # degrees
        self.min_clearance = 0.15  # meters
        self.shelf_min_y = 0.0
        self.shelf_max_y = 9.1
        
        # State
        self.waypoints: List[Waypoint] = []
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
        
        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
    
    def print_instructions(self):
        """Print usage instructions."""
        print("\n" + "="*70)
        print("  INTERACTIVE WAYPOINT RECORDER")
        print("="*70)
        print(f"  Max waypoints: {self.max_waypoints}")
        print(f"  Wheelbase offset: {self.wheelbase_offset}m")
        print(f"  Valid wheel distance: {self.min_wheel_dist*100:.0f}-{self.max_wheel_dist*100:.0f}cm")
        print(f"  Parallel tolerance: ±{self.parallel_tol}°")
        print("="*70)
        print("\n  WALL CONFIGURATION:")
        print("    Waypoints 1-4:  NORTH WALL (parallel to X-axis, yaw ≈ 0° or ±180°)")
        print("    Waypoints 5-10: EAST WALL (parallel to Y-axis, yaw ≈ ±90°)")
        print("="*70)
        print("\n  CONTROLS:")
        print("    SPACEBAR - Validate and save current position")
        print("    d        - Delete last waypoint")
        print("    q        - Quit and save waypoints to file")
        print("\n  Waiting for robot data...")
        print("="*70 + "\n")
    
    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Store current robot pose."""
        self.current_pose = msg
    
    def scan_callback(self, msg: LaserScan) -> None:
        """Store latest laser scan."""
        self.latest_scan = msg
    
    def get_current_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current robot position (x, y, yaw).
        
        Returns:
            Tuple of (x, y, yaw) or None if not available
        """
        # Try AMCL pose first
        if self.current_pose is not None:
            pose = self.current_pose.pose.pose
            quat = pose.orientation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (pose.position.x, pose.position.y, yaw)
        
        # Fallback to TF (SLAM)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            quat = transform.transform.rotation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (x, y, yaw)
        except:
            return None
    
    def measure_distance_to_rack(self) -> Optional[Tuple[float, float]]:
        """
        Measure distance from base_link and left wheels to rack on LEFT SIDE.
        
        IMPORTANT: Bot drives PARALLEL to rack with rack on LEFT side.
        lidar_1 is rotated 180° in the robot frame:
        - angle ~π/2 (90°) points LEFT in robot frame
        
        Robot geometry:
        - Lidar at y=0.0 (centered on base_link)
        - Left wheel at y=0.212m (21.2cm to the left of base_link)
        - So left wheel is 21.2cm CLOSER to rack than lidar measures
        
        Returns:
            (base_link_distance, left_wheel_distance) or None
        """
        if self.latest_scan is None:
            return None
        
        # Left side is at angle ~3π/2 (270°) in lidar frame due to 180° rotation
        # Which is π/2 (90°) in robot frame
        center_angle = 3 * math.pi / 2  # 270° in lidar frame = left in robot frame
        angle_range = math.radians(15)
        left_readings = []
        
        for i, distance in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
            
            # Check if angle is within ±15° of 3π/2 (left direction)
            angle_diff = abs(angle - center_angle)
            # Handle wrap-around at 2π
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff <= angle_range:
                if (self.latest_scan.range_min < distance < self.latest_scan.range_max 
                    and not math.isinf(distance)):
                    left_readings.append(distance)
        
        if not left_readings:
            return None
        
        # Lidar reading to rack
        lidar_to_rack = min(left_readings)
        
        # Base_link is at same lateral position as lidar (y=0)
        base_link_dist = lidar_to_rack
        
        # Left wheel is 21.2cm to the left of base_link, so CLOSER to rack
        left_wheel_offset = 0.212  # meters
        left_wheel_dist = base_link_dist - left_wheel_offset
        
        return (base_link_dist, left_wheel_dist)
    
    def estimate_parallel_angle(self, waypoint_num: int, robot_yaw: float) -> Optional[float]:
        """
        Estimate angle between robot and rack (how parallel).
        
        Waypoints 1-4: Should be parallel to x-axis (yaw ≈ 0° or ±180°)
        Waypoints 5-10: Should be parallel to y-axis (yaw ≈ ±90°)
        
        Args:
            waypoint_num: Current waypoint number (1-10)
            robot_yaw: Current robot yaw in radians
            
        Returns:
            Angle error in degrees, or None
        """
        if self.latest_scan is None:
            return None
        
        # Determine expected orientation based on waypoint number
        robot_yaw_deg = math.degrees(robot_yaw)
        
        if waypoint_num <= 4:
            # North wall (parallel to x-axis): yaw should be ~0° or ~180°
            # Normalize to closest 0° or 180°
            if abs(robot_yaw_deg) < 90:
                expected_yaw = 0.0
            else:
                expected_yaw = 180.0 if robot_yaw_deg > 0 else -180.0
            
            angle_error = robot_yaw_deg - expected_yaw
            
        else:
            # East wall (parallel to y-axis): yaw should be ~90° or ~-90°
            # Normalize to closest ±90°
            if robot_yaw_deg > 0:
                expected_yaw = 90.0
            else:
                expected_yaw = -90.0
            
            angle_error = robot_yaw_deg - expected_yaw
        
        # Normalize angle error to [-180, 180]
        while angle_error > 180:
            angle_error -= 360
        while angle_error < -180:
            angle_error += 360
        
        return angle_error
    
    def measure_lateral_clearance(self) -> float:
        """Measure clearance on sides."""
        if self.latest_scan is None:
            return 0.0
        
        # Check sides (60-120° left and right)
        left_min = float('inf')
        right_min = float('inf')
        
        for i, distance in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
            
            if (self.latest_scan.range_min < distance < self.latest_scan.range_max 
                and not math.isinf(distance)):
                
                if 1.0 < angle < 2.1:  # Left side
                    left_min = min(left_min, distance)
                elif -2.1 < angle < -1.0:  # Right side
                    right_min = min(right_min, distance)
        
        return min(left_min, right_min) if left_min != float('inf') else 0.0
    
    def validate_and_save_waypoint(self) -> bool:
        """
        Validate current position and save as waypoint.
        
        Returns:
            True if successful, False otherwise
        """
        print("\n" + "-"*70)
        print(f"  VALIDATING WAYPOINT #{len(self.waypoints) + 1}")
        print("-"*70)
        
        # Get position
        position = self.get_current_position()
        if position is None:
            print("  ✗ ERROR: Cannot get robot position")
            return False
        
        x, y, yaw = position
        waypoint_num = len(self.waypoints) + 1
        
        # Show which wall this waypoint is for
        if waypoint_num <= 4:
            wall_name = "NORTH WALL (parallel to X-axis)"
            expected_orientation = "0° or ±180°"
        else:
            wall_name = "EAST WALL (parallel to Y-axis)"
            expected_orientation = "±90°"
        
        print(f"  Wall: {wall_name}")
        print(f"  Expected orientation: {expected_orientation}")
        print(f"  Position: x={x:.3f}m, y={y:.3f}m, yaw={math.degrees(yaw):.1f}°")
        
        # Check distance only for odd-numbered waypoints (1,3,5,7,9)
        if waypoint_num % 2 == 1:  # Odd waypoints
            # Measure distances
            distances = self.measure_distance_to_rack()
            if distances is None:
                print("  ✗ ERROR: Cannot measure distance to rack")
                return False
            
            base_link_dist, front_wheel_dist = distances
            print(f"  Left side distance (base_link): {base_link_dist*100:.1f}cm")
            print(f"  Left wheel distance: {front_wheel_dist*100:.1f}cm")
            
            # Check front wheel distance
            if not (self.min_wheel_dist <= front_wheel_dist <= self.max_wheel_dist):
                print(f"  ✗ FAIL: Left wheel distance outside range "
                      f"{self.min_wheel_dist*100:.0f}-{self.max_wheel_dist*100:.0f}cm")
                if front_wheel_dist < self.min_wheel_dist:
                    print(f"      → TOO CLOSE - move RIGHT (away from rack) {(self.min_wheel_dist - front_wheel_dist)*100:.1f}cm")
                else:
                    print(f"      → TOO FAR - move LEFT (toward rack) {(front_wheel_dist - self.max_wheel_dist)*100:.1f}cm")
                return False
            
            print(f"  ✓ Left wheel distance OK")
        else:  # Even waypoints - skip distance check
            print(f"  ⊘ Distance check SKIPPED (even waypoint)")
            base_link_dist = 0.0
            front_wheel_dist = 0.0
        
        # Check parallel alignment (wall-specific)
        parallel_angle = self.estimate_parallel_angle(waypoint_num, yaw)
        if parallel_angle is not None:
            print(f"  Orientation error: {parallel_angle:.1f}° (from expected)")
            if abs(parallel_angle) > self.parallel_tol:
                print(f"  ✗ FAIL: Not parallel to wall (tolerance ±{self.parallel_tol}°)")
                if parallel_angle > 0:
                    print(f"      → Rotate RIGHT {abs(parallel_angle):.1f}°")
                else:
                    print(f"      → Rotate LEFT {abs(parallel_angle):.1f}°")
                return False
            print(f"  ✓ Parallel alignment OK")
        
        # Check clearance
        clearance = self.measure_lateral_clearance()
        print(f"  Lateral clearance: {clearance*100:.1f}cm")
        if clearance < self.min_clearance:
            print(f"  ⚠ WARNING: Low clearance (<{self.min_clearance*100:.0f}cm)")
        else:
            print(f"  ✓ Clearance OK")
        
        # Check shelf bounds
        if not (self.shelf_min_y <= y <= self.shelf_max_y):
            print(f"  ✗ FAIL: Outside shelf bounds ({self.shelf_min_y:.1f}-{self.shelf_max_y:.1f}m)")
            return False
        
        print(f"  ✓ Within shelf bounds")
        
        # Create waypoint
        waypoint_id = len(self.waypoints) + 1
        rack_id = f"R{waypoint_id:02d}"
        
        # Set descriptive name based on wall
        if waypoint_id <= 4:
            wall_type = "North"
        else:
            wall_type = "East"
        
        validation = ValidationMetadata(
            front_wheel_distance=front_wheel_dist,  # Actually left wheel
            base_link_distance=base_link_dist,
            parallel_angle_deg=parallel_angle if parallel_angle is not None else 0.0,
            obstacle_clearance=clearance,
            timestamp=datetime.now().isoformat(),
            operator_notes=f"{wall_type} wall waypoint {waypoint_id} - lateral distance to rack"
        )
        
        waypoint = Waypoint(
            id=waypoint_id,
            name=f"Rack {rack_id}",
            rack_id=rack_id,
            x=x,
            y=y,
            z=0.0,
            yaw=yaw,
            validation=asdict(validation)
        )
        
        self.waypoints.append(waypoint)
        
        print("-"*70)
        print(f"  ✅ WAYPOINT {waypoint_id} SAVED: {rack_id}")
        print(f"  Progress: {len(self.waypoints)}/{self.max_waypoints} waypoints")
        print("-"*70 + "\n")
        
        return True
    
    def save_to_file(self, filename: str = None):
        """Save waypoints to YAML file."""
        if filename is None:
            filename = '/home/b24me1066/Warehouse_Simulation/config/waypoints_competition.yaml'
        
        if not self.waypoints:
            print("No waypoints to save!")
            return
        
        data = {
            'waypoints': [asdict(wp) for wp in self.waypoints]
        }
        
        try:
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            with open(filename, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            print("\n" + "="*70)
            print(f"  ✅ SAVED {len(self.waypoints)} WAYPOINTS")
            print(f"  File: {filename}")
            print("="*70 + "\n")
        except Exception as e:
            print(f"\n✗ ERROR saving file: {e}\n")
    
    def get_key(self):
        """Get a single keypress."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            return key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return None
    
    def cleanup(self):
        """Restore terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main():
    rclpy.init()
    
    recorder = InteractiveWaypointRecorder(max_waypoints=10)
    
    # Give TF buffer time to fill
    print("  Initializing TF buffer...")
    for _ in range(20):  # Spin for ~2 seconds
        rclpy.spin_once(recorder, timeout_sec=0.1)
    
    # Wait for initial data
    print("  Waiting for robot pose and scan data...")
    start_time = recorder.get_clock().now()
    data_ready = False
    status_printed = False
    
    while rclpy.ok():
        rclpy.spin_once(recorder, timeout_sec=0.1)
        
        position = recorder.get_current_position()
        
        # Debug status every 2 seconds
        elapsed = (recorder.get_clock().now() - start_time).nanoseconds / 1e9
        if not status_printed and elapsed > 2.0:
            print(f"  Status: pose={'OK' if position else 'WAITING'}, scan={'OK' if recorder.latest_scan else 'WAITING'}")
            status_printed = True
        
        if position is not None and recorder.latest_scan is not None:
            if not data_ready:
                print("  ✓ Robot data ready!\n")
                if recorder.current_pose is not None:
                    print("  Using: AMCL localization\n")
                else:
                    print("  Using: TF/SLAM localization\n")
                print("  Press SPACEBAR to record waypoint, 'q' to quit\n")
                data_ready = True
            break
        
        if elapsed > 15.0:
            print("  ✗ Timeout waiting for robot data")
            print("  Make sure SLAM or AMCL is running!")
            print(f"  Final status: pose={'OK' if position else 'FAIL'}, scan={'OK' if recorder.latest_scan else 'FAIL'}")
            recorder.cleanup()
            recorder.destroy_node()
            rclpy.shutdown()
            return
    
    # Main loop
    try:
        last_status_time = recorder.get_clock().now()
        
        while rclpy.ok() and len(recorder.waypoints) < recorder.max_waypoints:
            rclpy.spin_once(recorder, timeout_sec=0.01)
            
            # Show real-time distance every 0.5 seconds
            current_time = recorder.get_clock().now()
            elapsed_since_status = (current_time - last_status_time).nanoseconds / 1e9
            
            if elapsed_since_status > 0.5:
                distances = recorder.measure_distance_to_rack()
                if distances:
                    base_dist, wheel_dist = distances
                    status = "✓ OK" if 0.15 <= wheel_dist <= 0.25 else "✗ OUT OF RANGE"
                    print(f"\r  Live: Left wheel={wheel_dist*100:.1f}cm (base_link={base_dist*100:.1f}cm) {status}   ", end='', flush=True)
                last_status_time = current_time
            
            key = recorder.get_key()
            
            if key == ' ':  # Spacebar
                print()  # New line after live display
                recorder.validate_and_save_waypoint()
                
                if len(recorder.waypoints) >= recorder.max_waypoints:
                    print(f"\n  Maximum waypoints ({recorder.max_waypoints}) reached!")
                    break
            
            elif key == 'd' or key == 'D':  # Delete last waypoint
                print()  # New line after live display
                if recorder.waypoints:
                    deleted = recorder.waypoints.pop()
                    print("\n" + "="*70)
                    print(f"  🗑️  DELETED WAYPOINT: {deleted.rack_id} ({deleted.name})")
                    print(f"  Remaining waypoints: {len(recorder.waypoints)}/{recorder.max_waypoints}")
                    print("="*70 + "\n")
                else:
                    print("\n  ⚠️  No waypoints to delete!\n")
            
            elif key == 'q' or key == 'Q':
                print("\n  Quitting...")
                break
    
    except KeyboardInterrupt:
        print("\n  Interrupted by user")
    
    finally:
        # Save waypoints
        if recorder.waypoints:
            recorder.save_to_file()
        else:
            print("\n  No waypoints recorded.")
        
        recorder.cleanup()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
