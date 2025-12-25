#!/usr/bin/env python3
"""
Competition Mission Launch File

Launches complete warehouse robot competition system:
- Nav2 stack with competition parameters
- Obstacle speed controller
- Emergency stop system
- WiFi monitor
- Competition waypoint navigator
- Vertical scanner
- QR code scanner
- Web server

Author: Competition Team
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate competition mission launch description."""
    
    # Package directories
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    pkg_scuttle_nav = FindPackageShare('scuttle_navigation2')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    resume_checkpoint = LaunchConfiguration('resume')
    enable_rviz = LaunchConfiguration('rviz')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_scuttle_nav,
            'config',
            'nav2_params_competition.yaml'
        ]),
        description='Full path to competition Nav2 parameters file'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='/home/b24me1066/Warehouse_Simulation/maps/warehouse_map.yaml',
        description='Full path to map file'
    )
    
    declare_resume = DeclareLaunchArgument(
        'resume',
        default_value='False',
        description='Resume from checkpoint'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz'
    )
    
    # ==========================================================================
    # NAV2 STACK
    # ==========================================================================
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_nav2_bringup,
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_file,
            'autostart': 'True'
        }.items()
    )
    
    # ==========================================================================
    # SAFETY SYSTEMS
    # ==========================================================================
    
    # Obstacle-based speed controller (DISABLED - causes random stops and Nav2 conflicts)
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
    
    # WiFi connection monitor (DISABLED - remove WiFi dependency)
    # wifi_monitor = Node(
    #     package='scuttle_navigation2',
    #     executable='wifi_monitor.py',
    #     name='wifi_monitor',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'target_host': '192.168.1.1',
    #         'ping_interval': 2.0,
    #         'failure_threshold': 3,
    #         'enable_failsafe': True
    #     }]
    # )
    
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
            'max_scan_duration_sec': 180.0
        }]
    )
    
    # QR code scanner
    qr_scanner = Node(
        package='scuttle_navigation2',
        executable='qr_code_scanner.py',
        name='qr_code_scanner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_device': 0,
            'resolution_width': 1920,
            'resolution_height': 1080,
            'fps': 30,
            'scan_duration_sec': 3.0,
            'enable_display': False
        }]
    )
    
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
            'rack_distance_min_m': 0.10,
            'rack_distance_max_m': 0.25,
            'parallel_tolerance_deg': 5.0
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
            'rack_distance_min_m': 0.10,
            'rack_distance_max_m': 0.25,
            'parallel_tolerance_deg': 5.0
        }],
        condition=UnlessCondition(resume_checkpoint)
    )
    
    # ==========================================================================
    # WEB INTERFACE
    # ==========================================================================
    
    # Flask web server
    web_server = ExecuteProcess(
        cmd=['python3', 
             os.path.join(
                 os.path.expanduser('~'),
                 'eternal_ws/src/scuttle_navigation2/scripts/competition_server.py'
             )],
        output='screen',
        name='competition_web_server'
    )
    
    # ==========================================================================
    # VISUALIZATION
    # ==========================================================================
    
    # RViz (optional)
    rviz_config_file = PathJoinSubstitution([
        pkg_scuttle_nav,
        'rviz',
        'competition_nav.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_rviz)
    )
    
    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_params_file,
        declare_map_file,
        declare_resume,
        declare_rviz,
        
        # Nav2 stack
        nav2_bringup,
        
        # Safety systems
        # obstacle_speed_controller,  # Disabled - causes random stops and Nav2 conflicts
        emergency_stop,
        # wifi_monitor,  # Disabled - WiFi dependency removed
        
        # Scanning system
        vertical_scanner,
        qr_scanner,
        
        # Mission control
        waypoint_navigator_resume,
        waypoint_navigator_normal,
        
        # Web interface
        web_server,
        
        # Visualization
        rviz
    ])
