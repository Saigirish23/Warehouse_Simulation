#!/usr/bin/env python3
"""
Localization Launch with Keepout Filter Integration
Extends the original localization_launch.py to include keepout filter nodes
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('scuttle_navigation2')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_keepout = LaunchConfiguration('use_keepout')
    
    # Lifecycle nodes - now includes keepout filter servers
    lifecycle_nodes = ['map_server', 'amcl']
    keepout_lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env variables
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Declare arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'warehouse_map.yaml'),
            description='Full path to map yaml file to load'),
        
        DeclareLaunchArgument(
            'keepout_mask',
            default_value='/home/b24me1066/Warehouse_Simulation/maps/keepout_mask.yaml',
            description='Full path to keepout mask yaml file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),
        
        DeclareLaunchArgument(
            'use_keepout',
            default_value='true',
            description='Whether to launch keepout filter nodes'),

        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        # AMCL Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        
        # Keepout Filter Mask Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            output='screen',
            parameters=[
                {'yaml_filename': keepout_mask_yaml_file},
                {'topic_name': 'keepout_filter_mask'},
                {'frame_id': 'map'},
                {'use_sim_time': use_sim_time}
            ],
            remappings=remappings),
        
        # Keepout Costmap Filter Info Server
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            parameters=[
                {'type': 0},
                {'filter_info_topic': 'costmap_filter_info'},
                {'mask_topic': 'keepout_filter_mask'},
                {'base': 0.0},
                {'multiplier': 1.0},
                {'use_sim_time': use_sim_time}
            ],
            remappings=remappings),

        # Lifecycle Manager for localization nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
        
        # Lifecycle Manager for keepout filter nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': keepout_lifecycle_nodes}]),
    ])
