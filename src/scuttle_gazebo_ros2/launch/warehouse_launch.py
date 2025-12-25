#!/usr/bin/env python3
"""
ROS2 Launch file for SCUTTLE in Gazebo Warehouse
Based on TurtleBot3 Gazebo launch pattern
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Package directories
    launch_file_dir = os.path.join(get_package_share_directory('scuttle_gazebo_ros2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    scuttle_description_dir = get_package_share_directory('scuttle_description_ros2')
    
    # World file
    world = os.path.join(
        get_package_share_directory('scuttle_gazebo_ros2'),
        'worlds',
        'warehouse.world'
    )
    
    # CRITICAL: Set Gazebo model path so Gazebo can resolve package:// URIs for meshes
    gazebo_model_path = os.path.join(scuttle_description_dir, 'meshes') + ':' + \
                        os.path.dirname(scuttle_description_dir) + ':' + \
                        os.environ.get('GAZEBO_MODEL_PATH', '')

    return LaunchDescription([
        # Set environment variable for Gazebo to find meshes
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        
        # Start Gazebo server with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world, 'verbose': 'true'}.items(),
        ),

        # Start Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={'verbose': 'true'}.items(),
        ),

        # Robot State Publisher (separate launch file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])