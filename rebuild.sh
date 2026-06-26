#!/bin/bash
# Rebuild delivery robot packages and restart the running system.
#
# Usage:
#   ~/rebuild.sh                  # rebuild all delivery robot packages
#   ~/rebuild.sh robot_web_ui     # rebuild only one package
#
# After building, kills the running robot processes and relaunches
# start_robot.sh so the new code takes effect immediately.

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

ALL_PKGS="delivery_robot_msgs pi_camera_driver motor_driver aruco_detector delivery_mission robot_web_ui delivery_robot_bringup"
PKGS="${*:-$ALL_PKGS}"

echo "=== Rebuilding: $PKGS ==="

# Clean env: only base ROS sourced, so no other workspaces leak into the chain
env -i HOME="$HOME" PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  PKGS="$PKGS" WS_DIR="$WS_DIR" \
  bash -c '
    source /opt/ros/jazzy/setup.bash
    cd "$WS_DIR"
    colcon build --packages-select $PKGS
  '

echo ""
echo "=== Build complete, restarting robot ==="

pkill -f "rpicam-vid|motor_driver_node|web_ui_node|camera_node|aruco_detector_node|aruco_approach_node" 2>/dev/null || true
fuser -k 8080/tcp 2>/dev/null || true
sleep 1

sg dialout -c "bash -c '
  source /opt/ros/jazzy/setup.bash
  source \"$WS_DIR/install/setup.bash\"
  ros2 launch delivery_robot_bringup bringup.launch.py
'"
