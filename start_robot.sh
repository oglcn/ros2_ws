#!/bin/bash
# Launch the delivery robot system with full log output.
# Usage: ~/start_robot.sh
#
# Ctrl+C to stop all nodes.

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# Kill any stale processes from a previous run
pkill -f "rpicam-vid|motor_driver_node|web_ui_node|camera_node|aruco_detector_node|aruco_approach_node|imu_driver_node" 2>/dev/null || true
fuser -k 8080/tcp 2>/dev/null || true
sleep 1

# Reset the I2C-1 controller to clear any stuck state from a previous crash.
# This fixes controller-side hangs. If the sensor itself is holding SDA low,
# only a power cycle of the MPU6050 (or reboot) can clear it.
I2C_DEV="1f00074000.i2c"
I2C_DRV="/sys/bus/platform/drivers/i2c_designware"
if [ -e "$I2C_DRV/$I2C_DEV" ]; then
  echo "$I2C_DEV" | sudo tee "$I2C_DRV/unbind" >/dev/null 2>&1 || true
  sleep 0.3
  echo "$I2C_DEV" | sudo tee "$I2C_DRV/bind" >/dev/null 2>&1 || true
  sleep 0.3
  echo "I2C bus 1 reset"
fi

# Launch in a clean env so no other workspace leaks into AMENT_PREFIX_PATH
# sg dialout = GPIO access, sg i2c = IMU I2C access
exec env -i \
  HOME="$HOME" \
  USER="$USER" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  TERM="${TERM:-xterm}" \
  sg dialout -c "
    sg i2c -c '
      bash -c \"source /opt/ros/jazzy/setup.bash && source $WS_DIR/install/setup.bash && ros2 launch delivery_robot_bringup bringup.launch.py\"
    '
  "
