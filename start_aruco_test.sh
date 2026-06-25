#!/bin/bash
# Launch camera + ArUco detector only (no motor control) for bench-testing
# marker detection. Open http://<robot-ip>:8081 and enter a marker ID to
# see live whether the camera currently sees it.
# Usage: ~/start_aruco_test.sh
#
# Ctrl+C to stop all nodes.

set -e

# Workspace = the directory this script lives in (so moving the repo just works)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# Kill any stale processes from a previous run
pkill -f "rpicam-vid|aruco_detector_node|aruco_test_node|camera_node" 2>/dev/null || true
fuser -k 8081/tcp 2>/dev/null || true
sleep 1

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
ros2 launch delivery_robot_bringup aruco_test.launch.py
