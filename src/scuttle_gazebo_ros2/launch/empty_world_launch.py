#!/usr/bin/env python3
"""
Empty World Launch for SCUTTLE - Clean test environment
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Get the launch directory
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    scuttle_description_dir = get_package_share_directory('scuttle_description_ros2')
    
    # CRITICAL: Set GAZEBO_MODEL_PATH so Gazebo can resolve package:// URIs
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(scuttle_description_dir, 'meshes') + ':' + 
              os.path.dirname(scuttle_description_dir) + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    
    # Gazebo launch - EMPTY WORLD
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'pause': 'false',
            'gui': gui
        }.items()
    )
    
    # Process robot URDF
    scuttle_description_dir = get_package_share_directory('scuttle_description_ros2')
    xacro_file = os.path.join(scuttle_description_dir, 'urdf', 'scuttle.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toprettyxml(indent='  ')
    
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
    
    # Spawn SCUTTLE robot (delayed to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_scuttle',
            output='screen',
            arguments=[
                '-entity', 'scuttle',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
                '-Y', '0.0'
            ]
        )]
    )
    
    # Spawn two cylinders as obstacles (radius=0.05m, height=1m)
    spawn_cylinder_1 = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'cylinder_1',
                 '-x', '2.0', '-y', '1.0', '-z', '0.5',
                 '-database', 'cylinder'],
            output='screen'
        )]
    )
    
    spawn_cylinder_2 = TimerAction(
        period=5.5,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'cylinder_2',
                 '-x', '2.0', '-y', '-1.0', '-z', '0.5',
                 '-database', 'cylinder'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_robot,
        spawn_cylinder_1,
        spawn_cylinder_2,
    ])
