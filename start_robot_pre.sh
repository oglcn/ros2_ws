#!/bin/bash
# Pre-start cleanup for the delivery robot systemd service.
# Runs as root (via systemd ExecStartPre=+) to reset I2C and kill stale processes.

pkill -f "rpicam-vid|motor_driver_node|web_ui_node|camera_node|aruco_detector_node|aruco_approach_node|imu_driver_node" 2>/dev/null || true
fuser -k 8080/tcp 2>/dev/null || true
sleep 1

# Reset I2C-1 controller to clear stuck state from a previous crash
I2C_DEV="1f00074000.i2c"
I2C_DRV="/sys/bus/platform/drivers/i2c_designware"
if [ -e "$I2C_DRV/$I2C_DEV" ]; then
  echo "$I2C_DEV" > "$I2C_DRV/unbind" 2>/dev/null || true
  sleep 0.3
  echo "$I2C_DEV" > "$I2C_DRV/bind" 2>/dev/null || true
  sleep 0.3
  echo "I2C bus 1 reset"
fi
