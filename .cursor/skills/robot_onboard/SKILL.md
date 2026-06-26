---
name: robot-onboard
description: >-
  Onboarding for the delivery robot project. Use when working on any file in
  ros2_ws/src/delivery_robot/, or when the user mentions the robot, motor
  driver, camera driver, web UI, ArUco, VSLAM, IMU, MPU6050, or mecanum wheels.
---

# Delivery Robot -- Agent Onboarding

## System

- Raspberry Pi 5 (8 GB), Ubuntu 24.04, ROS2 Jazzy
- Pi Camera Module 3 (IMX708), 2x L298N motor drivers, 4x mecanum wheels
- Workspace: `~/ros2_ws`, monorepo at `ros2_ws/src/delivery_robot/`

## Package Map

| Package | Main source | What it does |
|---|---|---|
| `pi_camera_driver` | `pi_camera_driver/camera_node.py` | Launches `rpicam-vid` subprocess, publishes CompressedImage, raw Image, and CameraInfo |
| `motor_driver` | `motor_driver/motor_driver_node.py` | `/cmd_vel` subscriber, mecanum kinematics, L298N PWM via `lgpio` |
| `robot_web_ui` | `robot_web_ui/web_ui_node.py` | aiohttp on port 8080: MJPEG stream, WebSocket bridge, static HTML/JS frontend |
| `delivery_robot_msgs` | `msg/MotorStatus.msg` | Custom message: `float32[4]`, `bool[4]`, `string` |
| `delivery_robot_bringup` | `launch/bringup.launch.py` | Launch files + YAML config in `config/` |
| `aruco_detector` | `aruco_detector/aruco_detector_node.py` | Detects ArUco markers, publishes PoseWithCovarianceStamped + TF |
| `imu_driver` | `imu_driver/imu_driver_node.py` | MPU6050 I2C driver: publishes `sensor_msgs/Imu` at 50 Hz to `/imu/data_raw` |
| `orb_slam3_ros` | `src/orb_slam3_node.cpp` | ORB-SLAM3 monocular wrapper: publishes `/visual_odom` + TF `odom→base_link` |

Frontend: `robot_web_ui/robot_web_ui/static/index.html` -- vanilla JS, no build step.

## Critical Gotchas

1. **libcamera env isolation**: `rpicam-vid` MUST run in a minimal env that strips `LD_LIBRARY_PATH`, `AMENT_PREFIX_PATH`, and `PYTHONPATH`. ROS2's libcamera lacks PiSP support. See `camera_node.py:_build_clean_env()`.

2. **gpiochip4**: Pi 5 uses gpiochip4 (RP1) for the 40-pin header. Always use `lgpio.gpiochip_open(4)`. Configured via `gpio_chip` param in `motor_pins.yaml`.

3. **dialout group**: GPIO access requires the `dialout` group. The launch script uses `sg dialout`. If you get "can not open gpiochip", check group membership.

3b. **i2c group**: IMU (I2C) access requires the `i2c` group. Run `sudo usermod -aG i2c pi` and re-login. If you get "Permission denied" on `/dev/i2c-1`, check group membership.

4. **ROS2 fixed-size arrays in Python**: Do NOT assign a list to `msg.duty_cycles` or `msg.active`. Assign each element explicitly:
   ```python
   for i in range(4):
       msg.duty_cycles[i] = float(values[i])
       msg.active[i] = bool(flags[i])
   ```

5. **JSON serialization**: ROS2 `float32` and `bool` are numpy types. Always convert with `float()` / `bool()` before `json.dumps()` or `ws.send_json()`.

6. **Static files packaging**: `robot_web_ui/setup.py` uses `package_data` (not just `data_files`) to include `static/` in the installed Python package. New static files are picked up automatically.

7. **ORB-SLAM3 shared libraries**: `libORB_SLAM3.so`, `libDBoW2.so`, and `libg2o.so` live in `/home/pi/third_party/ORB_SLAM3/`. Their paths are in `/etc/ld.so.conf.d/orbslam3.conf`. If you get "cannot open shared object", run `sudo ldconfig`.

8. **OpenCV 4.6 ArUco API**: System OpenCV is 4.6 which uses the older `cv2.aruco.detectMarkers()` module-level function and `DetectorParameters_create()`. Do NOT use the newer `ArucoDetector` class (4.7+).

## Build and Run

```bash
# Full build
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build

# Incremental (single package)
colcon build --packages-select <package_name>

# Launch (base system)
~/start_robot.sh

# Or manually:
sg dialout -c 'bash -c "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch delivery_robot_bringup bringup.launch.py"'

# Launch localization stack (in a separate terminal)
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch delivery_robot_bringup localization.launch.py

# Launch localization without ArUco
ros2 launch delivery_robot_bringup localization.launch.py use_aruco:=false

# Camera calibration (requires checkerboard)
ros2 run pi_camera_driver calibrate_camera -- --rows 7 --cols 9 --square-size 0.025
```

## Config Files

| File | Path | Key params |
|---|---|---|
| Motor pins | `delivery_robot_bringup/config/motor_pins.yaml` | GPIO pins, `min_duty`, `max_speed`, `pwm_frequency`, `watchdog_timeout` |
| Camera | `delivery_robot_bringup/config/camera.yaml` | `width`, `height`, `framerate`, `quality`, `publish_raw`, `calibration_file` |
| Camera calibration | `delivery_robot_bringup/config/camera_calibration.yaml` | Intrinsics (K, D, R, P matrices) |
| ArUco markers | `delivery_robot_bringup/config/aruco_markers.yaml` | `marker_size`, `dictionary`, marker world positions |
| IMU | `delivery_robot_bringup/config/imu.yaml` | I2C bus/address, publish rate, DLPF, calibration offsets |
| EKF | `delivery_robot_bringup/config/ekf.yaml` | `robot_localization` config: odom0, pose0, imu0, covariances |
| ORB-SLAM3 | `orb_slam3_ros/config/orb_slam3_pi5.yaml` | Camera intrinsics, ORB features (600), scale levels (6) |

After editing config, rebuild `delivery_robot_bringup` and restart.

## Deeper Context

For full details, read these files in the project:

- [README.md](ros2_ws/src/delivery_robot/README.md) -- architecture, topics, web UI controls
- [HARDWARE.md](ros2_ws/src/delivery_robot/HARDWARE.md) -- GPIO pinout, wiring, power, mecanum layout
- [ROADMAP.md](ros2_ws/src/delivery_robot/ROADMAP.md) -- phases, open decisions, what's next
