#!/bin/bash

# ==========================================
# GSbot Startup Script
# ==========================================

# 1. Load Environment Variables
# (Crucial for systemd service to find ROS commands)
source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
source $HOME/Repos/gsbot/venv-ubuntu/bin/activate
export ROS_DOMAIN_ID=0  # Default ID

# Navigate to workspace
cd $HOME/Repos/gsbot

# 2. Start ROS Bridge (The WebSocket Communication)
# We use '&' to run it in the background
echo "Starting Rosbridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
PID_BRIDGE=$!
sleep 5  # Wait for bridge to initialize

# 3. Start the Hardware Driver (The Python Node)
echo "Starting Dual-Mode Driver..."
python3 drive_node.py &
PID_DRIVER=$!
sleep 2

# 4. Start the ttyd server
echo "Starting ttyd Server..."
./ttyd.x86_64 --writable -p 8080 bash &
PID_DRIVER=$!
sleep 2

# 5. Start the Flask Web Server (The Brain & UI)
# This is the main process, so we keep it in foreground (no '&')
# If Flask dies, the whole service restarts.
echo "Starting Flask Server..."
python3 web/app.py

# ==========================================
# Cleanup (If Flask stops, kill others)
# ==========================================
kill $PID_BRIDGE
kill $PID_DRIVER
