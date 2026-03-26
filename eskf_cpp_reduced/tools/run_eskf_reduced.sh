#!/bin/bash

# Setup core ROS 2 environment
source /opt/ros/humble/setup.bash

echo "Starting ESKF initialization sequence..."

# Camera and radar sensor initialization
echo "Starting camera and radar sensor initialization..."
(
    cd /home/ituarc/Documents/GitHub/Delayed-KalmanFilter-VisualGuidance/eskf_py/interceptor_sensor_emulators
    source install/setup.sh
    ros2 launch interceptor_sensor_emulators interceptor_emulators.launch.py
) &
SENSOR_PID=$!

# ESKF reduced initialization
echo "Starting ESKF reduced initialization..."
(
    source /home/ituarc/ros2_ws/install/setup.bash
    ros2 run eskf_cpp_reduced eskf_node_reduced --ros-args -p config_file:=/home/ituarc/ros2_ws/src/eskf_cpp_reduced/config/eskf_reduced_params.yaml
) &
ESKF_PID=$!

# Wait for background node processes so the service does not exit immediately
wait $SENSOR_PID $ESKF_PID
