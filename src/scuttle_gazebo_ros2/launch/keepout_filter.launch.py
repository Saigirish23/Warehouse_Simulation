#!/usr/bin/env python3
"""
Keepout Zone Launch for SCUTTLE with SLAM Navigation
Implements Nav2 Keepout Filter with proper lifecycle management
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace, SetParameter
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    # Get package directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    scuttle_nav_dir = get_package_share_directory('scuttle_navigation2')
    
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Lifecycle nodes for keepout filter
    lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']
    
    # Remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(scuttle_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask',
        default_value='/home/b24me1066/Warehouse_Simulation/maps/keepout_mask.yaml',
        description='Full path to keepout mask yaml file to load'
    )
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes'
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )
    
    # Load keepout filter nodes
    load_nodes = GroupAction(
        actions=[
            PushROSNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),
            
            # Map Server for keepout filter mask
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {'yaml_filename': keepout_mask_yaml_file},
                    {'topic_name': 'keepout_filter_mask'},
                    {'frame_id': 'map'},
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            
            # Costmap Filter Info Server for keepout zones
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='keepout_costmap_filter_info_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {'type': 0},  # 0 = Keepout filter
                    {'filter_info_topic': 'costmap_filter_info'},
                    {'mask_topic': 'keepout_filter_mask'},
                    {'base': 0.0},
                    {'multiplier': 1.0},
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            
            # Lifecycle manager for keepout filter nodes
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout_zone',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}
                ],
            ),
        ]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    
    # Add nodes
    ld.add_action(load_nodes)
    
    return ld
