#!/bin/bash
# ROS2 Startup Script for ESKF and Relative Position Node
# This script runs at system boot

# Wait for system to be ready
sleep 10

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/ituarc/ros2_ws/install/setup.bash

# Create log directory under eskf_cpp/log/service_log
LOG_DIR="/home/ituarc/ros2_ws/src/eskf_cpp/log/service_log"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Start ESKF launch in background with full output logging
echo "[$(date)] Starting ESKF..." | tee -a "$LOG_DIR/startup_$TIMESTAMP.log"
stdbuf -oL -eL ros2 launch eskf_cpp eskf.launch.py print_level:=INFO 2>&1 | tee "$LOG_DIR/eskf_$TIMESTAMP.log" &
ESKF_PID=$!
echo "[$(date)] ESKF started with PID: $ESKF_PID" | tee -a "$LOG_DIR/startup_$TIMESTAMP.log"

# Wait a bit for ESKF to initialize
sleep 5

# Start Relative Position Node in background with full output logging
echo "[$(date)] Starting Relative Position Node..." | tee -a "$LOG_DIR/startup_$TIMESTAMP.log"
stdbuf -oL -eL python3 -u /home/ituarc/ros2_ws/src/thermal_guidance/thermal_guidance/thermal_guidance/relative_position_node.py 2>&1 | tee "$LOG_DIR/relative_pos_$TIMESTAMP.log" &
RELPOS_PID=$!
echo "[$(date)] Relative Position Node started with PID: $RELPOS_PID" | tee -a "$LOG_DIR/startup_$TIMESTAMP.log"

# Keep script running
wait
