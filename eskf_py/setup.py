from setuptools import find_packages, setup

package_name = 'eskf_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    install_requires=['numpy', 'scipy', 'pyyaml'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/eskf_params.yaml']),
    ],
    entry_points={
        'console_scripts': [
            'eskf_node       = eskf_py.eskf_node:main',
            'eskf_test       = eskf_py.eskf_test_harness:main',
            'eskf_ros2_test  = eskf_py.eskf_ros2_test:main',
        ],
    },
)
