#!/usr/bin/env python3
"""
Competition Nodes Only Launch File

Launches only competition-specific nodes:
- Competition waypoint navigator
- Vertical scanner
- QR code scanner
- Obstacle speed controller
- Emergency stop system

User must launch Nav2/localization separately:
- ros2 launch scuttle_bringup_ros2 navigation_launch.py use_sim_time:=True

Author: Competition Team
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description with competition nodes only."""
    
    # Package directories
    pkg_scuttle_nav = FindPackageShare('scuttle_navigation2')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    resume_checkpoint = LaunchConfiguration('resume')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    declare_resume = DeclareLaunchArgument(
        'resume',
        default_value='False',
        description='Resume from checkpoint'
    )
    
    # ==========================================================================
    # SAFETY SYSTEMS
    # ==========================================================================
    
    # Obstacle-based speed controller (DISABLED - causes random stops)
    # obstacle_speed_controller = Node(
    #     package='scuttle_navigation2',
    #     executable='obstacle_speed_controller.py',
    #     name='obstacle_speed_controller',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'max_speed_open': 1.0,
    #         'max_speed_near_obstacle': 0.3,
    #         'safe_distance_threshold': 0.15,
    #         'full_speed_distance': 5.0,
    #         'update_rate': 10.0
    #     }]
    # )
    
    # Emergency stop system
    emergency_stop = Node(
        package='scuttle_navigation2',
        executable='emergency_stop.py',
        name='emergency_stop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_gpio': False,  # Set True for real hardware
            'monitor_rate': 50.0
        }]
    )
    
    # ==========================================================================
    # SCANNING SYSTEM
    # ==========================================================================
    
    # Vertical scanner for Z-axis camera control
    vertical_scanner = Node(
        package='scuttle_navigation2',
        executable='vertical_scanner.py',
        name='vertical_scanner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_height_cm': 20.0,
            'max_height_cm': 150.0,
            'height_increment_cm': 20.0,
            'positioning_tolerance_cm': 2.0,
            'max_scan_duration_sec': 180.0,
            'display_camera_feed': True
        }]
    )
    
    # QR code scanner (DISABLED - no camera in simulation)
    # qr_scanner = Node(
    #     package='scuttle_navigation2',
    #     executable='qr_code_scanner.py',
    #     name='qr_code_scanner',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'camera_device': 0,
    #         'resolution_width': 1920,
    #         'resolution_height': 1080,
    #         'fps': 30,
    #         'scan_duration_sec': 3.0,
    #         'enable_display': False
    #     }]
    # )
    
    # ==========================================================================
    # MISSION CONTROL
    # ==========================================================================
    
    # Competition waypoint navigator (with resume)
    waypoint_navigator_resume = Node(
        package='scuttle_navigation2',
        executable='competition_waypoint_navigator.py',
        name='competition_waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_mission_time_sec': 1200.0,  # 20 minutes
            'rack_distance_min_m': 0.15,
            'rack_distance_max_m': 0.25,
            'parallel_tolerance_deg': 5.0,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_yaw': 0.0
        }],
        arguments=['--resume'],
        condition=IfCondition(resume_checkpoint)
    )
    
    # Competition waypoint navigator (normal start)
    waypoint_navigator_normal = Node(
        package='scuttle_navigation2',
        executable='competition_waypoint_navigator.py',
        name='competition_waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_mission_time_sec': 1200.0,  # 20 minutes
            'rack_distance_min_m': 0.15,
            'rack_distance_max_m': 0.25,
            'parallel_tolerance_deg': 5.0,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_yaw': 0.0
        }],
        condition=UnlessCondition(resume_checkpoint)
    )
    
    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_resume,
        
        # Safety systems
        # obstacle_speed_controller,  # Disabled - causes random stops
        emergency_stop,
        
        # Scanning system
        vertical_scanner,
        # qr_scanner,  # Disabled for simulation
        
        # Mission control
        waypoint_navigator_resume,
        waypoint_navigator_normal,
    ])
