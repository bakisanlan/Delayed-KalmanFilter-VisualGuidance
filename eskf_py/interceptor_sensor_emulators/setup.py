import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'interceptor_sensor_emulators'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ament resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Sensor emulator nodes for interceptor simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_emulator_node = interceptor_sensor_emulators.camera_emulator_node:main',
            'radar_emulator_node = interceptor_sensor_emulators.radar_emulator_node:main',
            'target_state_bridge = interceptor_sensor_emulators.target_state_bridge:main',
            'visualization_node = interceptor_sensor_emulators.visualization_node:main',
        ],
    },
)
