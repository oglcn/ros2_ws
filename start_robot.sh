#!/bin/bash
# Launch the delivery robot system with full log output.
# Usage: ~/start_robot.sh
#
# Ctrl+C to stop all nodes.

set -e

# Workspace = the directory this script lives in (so moving the repo just works)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# Kill any stale processes from a previous run
pkill -f "rpicam-vid|motor_driver_node|web_ui_node|camera_node" 2>/dev/null || true
fuser -k 8080/tcp 2>/dev/null || true
sleep 1

# Launch within the dialout group (required for GPIO access)
sg dialout -c "bash -c '
  source /opt/ros/jazzy/setup.bash
  source \"$WS_DIR/install/setup.bash\"
  ros2 launch delivery_robot_bringup bringup.launch.py
'"
