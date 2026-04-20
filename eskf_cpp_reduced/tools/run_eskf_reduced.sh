#!/bin/bash

# Setup core ROS 2 environment
source /opt/ros/humble/setup.bash

echo "Starting ESKF initialization sequence..."

# Cleanup function: kill all child processes on shutdown
cleanup() {
    echo "Received shutdown signal, stopping all child processes..."
    # Kill entire process group of each child (covers ros2 launch subprocesses)
    kill -TERM -$SENSOR_PID  2>/dev/null
    kill -TERM -$ESKF_PID    2>/dev/null
    kill -TERM -$RECORDER_PID 2>/dev/null
    # Also kill the direct PIDs in case process-group kill missed
    kill -TERM $SENSOR_PID $ESKF_PID $RECORDER_PID 2>/dev/null
    # Give processes a short time to exit gracefully, then force-kill
    sleep 2
    kill -KILL -$SENSOR_PID  2>/dev/null
    kill -KILL -$ESKF_PID    2>/dev/null
    kill -KILL -$RECORDER_PID 2>/dev/null
    kill -KILL $SENSOR_PID $ESKF_PID $RECORDER_PID 2>/dev/null
    wait
    echo "All child processes stopped."
    exit 0
}

# Trap SIGTERM (systemd stop) and SIGINT (Ctrl+C)
trap cleanup SIGTERM SIGINT

# Camera and radar sensor initialization
echo "Starting camera and radar sensor initialization..."
(
    cd /home/ituarc/Documents/Github/Delayed-KalmanFilter-VisualGuidance/eskf_py/interceptor_sensor_emulators
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

# Rosbag recorder initialization
echo "Starting rosbag recorder..."
(
    source /home/ituarc/ros2_ws/install/setup.bash
    python3 /home/ituarc/ros2_ws/src/eskf_cpp_reduced/tools/rosbag_recorder.py
) &
RECORDER_PID=$!

# Wait for background node processes so the service does not exit immediately
# Using a loop so that 'wait' is interruptible by the trap
while kill -0 $SENSOR_PID 2>/dev/null || kill -0 $ESKF_PID 2>/dev/null || kill -0 $RECORDER_PID 2>/dev/null; do
    wait -n 2>/dev/null
done
