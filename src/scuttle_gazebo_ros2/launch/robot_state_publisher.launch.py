#!/usr/bin/env python3
"""
Robot State Publisher Launch for SCUTTLE
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the URDF/xacro file
    scuttle_description_dir = get_package_share_directory('scuttle_description_ros2')
    xacro_file = os.path.join(scuttle_description_dir, 'urdf', 'scuttle.xacro')
    
    # Process the xacro file - keep package:// URIs as-is
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toprettyxml(indent='  ')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Spawn Entity node - spawn upright in open area of warehouse
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_scuttle',
        output='screen',
        arguments=[
            '-entity', 'scuttle',
            '-topic', 'robot_description',
            '-x', '3.1',      # User specified position
            '-y', '-0.4',     # User specified position
            '-z', '0.05',     # Spawn 5cm above ground
            '-R', '0.0',      # Roll = 0 (no tilt)
            '-P', '0.0',      # Pitch = 0 (no tilt)
            '-Y', '3.14159'   # Yaw = π (face west)
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
