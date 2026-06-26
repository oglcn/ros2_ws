#!/bin/bash
# Launch camera + ArUco detector + motor driver + approach controller.
# The robot will physically drive toward the given marker ID once it's
# visible. Open http://<robot-ip>:8081 to watch the camera/detections.
#
# Usage: ~/start_aruco_approach.sh [marker_id]
#   e.g. ~/start_aruco_approach.sh 2
#
# Ctrl+C to stop all nodes.

set -e

TARGET_ID="${1:-0}"

# Workspace = the directory this script lives in (so moving the repo just works)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# Kill any stale processes from a previous run
pkill -f "rpicam-vid|motor_driver_node|aruco_detector_node|aruco_approach_node|aruco_test_node|camera_node" 2>/dev/null || true
fuser -k 8081/tcp 2>/dev/null || true
sleep 1

# Launch within the dialout group (required for GPIO access)
sg dialout -c "bash -c '
  source /opt/ros/jazzy/setup.bash
  source \"$WS_DIR/install/setup.bash\"
  ros2 launch delivery_robot_bringup aruco_approach.launch.py target_marker_id:=${TARGET_ID}
'"
