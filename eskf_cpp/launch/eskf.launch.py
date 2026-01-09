#!/usr/bin/env python3
"""
ESKF Launch File
Launches the Error-State Kalman Filter node with configuration
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('eskf_cpp')
    
    # Default config file
    default_config = os.path.join(pkg_share, 'config', 'eskf_params.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to ESKF configuration YAML file'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/mavros/imu/data_raw',
        description='IMU topic name'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/yolo/target',
        description='Image features topic name'
    )
    
    radar_topic_arg = DeclareLaunchArgument(
        'radar_topic',
        default_value='/radar/pr',
        description='Radar measurement topic name'
    )
    
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/eskf/pose',
        description='Output pose topic name'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    # ESKF Node
    eskf_node = Node(
        package='eskf_cpp',
        executable='eskf_node',
        name='eskf_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
            'radar_topic': LaunchConfiguration('radar_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        config_file_arg,
        imu_topic_arg,
        image_topic_arg,
        radar_topic_arg,
        pose_topic_arg,
        log_level_arg,
        eskf_node
    ])
