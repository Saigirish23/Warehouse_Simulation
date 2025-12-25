#!/usr/bin/env python3
"""
SLAM and Navigation Launch for SCUTTLE Robot
Uses SLAM Toolbox for mapping and Nav2 for navigation
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes'
    )
    
    # SLAM Toolbox - Online Async mode for real-time mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'debug_logging': False,
                'throttle_scans': 1,
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
                'stack_size_to_use': 40000000,
                'enable_interactive_mode': True
            }
        ]
    )
    
    # Nav2 Bringup - Navigation stack
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': os.path.join(
                get_package_share_directory('scuttle_navigation2'),
                'config',
                'nav2_params.yaml'
            ) if os.path.exists(os.path.join(
                get_package_share_directory('scuttle_navigation2'),
                'config',
                'nav2_params.yaml'
            )) else ''
        }.items()
    )
    
    # RViz2 with Nav2 config
    rviz_config_file = os.path.join(
        nav2_bringup_dir,
        'rviz',
        'nav2_default_view.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        slam_toolbox_node,
        nav2_bringup_launch,
        rviz_node,
    ])
