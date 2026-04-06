import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('interceptor_sensor_emulators')
    
    camera_config = os.path.join(pkg_share, 'config', 'camera_config.yaml')
    radar_config = os.path.join(pkg_share, 'config', 'radar_emulator_params.yaml')
    bridge_config = os.path.join(pkg_share, 'config', 'target_state_bridge_params.yaml')

    camera_node = Node(
        package='interceptor_sensor_emulators',
        executable='camera_emulator_node',
        name='camera_emulator_node',
        output='screen',
        parameters=[camera_config],
    )

    radar_node = Node(
        package='interceptor_sensor_emulators',
        executable='radar_emulator_node',
        name='radar_emulator_node',
        output='screen',
        parameters=[radar_config],
    )

    bridge_node = Node(
        package='interceptor_sensor_emulators',
        executable='target_state_bridge',
        name='target_state_bridge',
        output='screen',
        parameters=[bridge_config],
    )

    visualization_node = Node(
        package='interceptor_sensor_emulators',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
        parameters=[camera_config],
    )

    return LaunchDescription([camera_node, radar_node, bridge_node, visualization_node])
