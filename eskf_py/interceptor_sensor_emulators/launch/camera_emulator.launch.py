"""Launch file for the camera emulator node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('interceptor_sensor_emulators')
    config_file = os.path.join(pkg_share, 'config', 'camera_config.yaml')

    camera_node = Node(
        package='interceptor_sensor_emulators',
        executable='camera_emulator_node',
        name='camera_emulator_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([camera_node])
