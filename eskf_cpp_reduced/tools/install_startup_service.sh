#!/bin/bash

# Ensure script is run with sudo for systemctl and cp Commands
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo"
  exit 1
fi

SERVICES_DIR="/etc/systemd/system"
SERVICE_NAME="run_eskf_reduced.service"
SERVICE_PATH="/home/ituarc/ros2_ws/src/eskf_cpp_reduced/tools/$SERVICE_NAME"
SCRIPT_PATH="/home/ituarc/ros2_ws/src/eskf_cpp_reduced/tools/run_eskf_reduced.sh"

echo "Making script executable..."
chmod +x "$SCRIPT_PATH"

echo "Copying service file to $SERVICES_DIR..."
cp "$SERVICE_PATH" "$SERVICES_DIR/"

echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "Enabling $SERVICE_NAME to run on boot..."
systemctl enable "$SERVICE_NAME"

echo "Starting $SERVICE_NAME..."
systemctl start "$SERVICE_NAME"

echo "Installation complete. You can check the status with: sudo systemctl status $SERVICE_NAME"
