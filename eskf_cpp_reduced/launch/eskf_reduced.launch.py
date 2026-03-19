#!/usr/bin/env python3
"""
Launch file for the reduced-state ESKF node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('eskf_cpp_reduced')
    default_config = os.path.join(pkg_share, 'config', 'eskf_reduced_params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to reduced ESKF configuration YAML file'
    )

    print_level_arg = DeclareLaunchArgument(
        'print_level',
        default_value='INFO',
        description='Reduced ESKF print level (ALL, DEBUG, INFO, WARNING, ERROR, SILENT)'
    )

    node = Node(
        package='eskf_cpp_reduced',
        executable='eskf_node_reduced',
        name='eskf_node_reduced',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'print_level': LaunchConfiguration('print_level'),
        }]
    )

    return LaunchDescription([
        config_file_arg,
        print_level_arg,
        node,
    ])
