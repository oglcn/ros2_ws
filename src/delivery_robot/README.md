# Delivery Robot

Indoor autonomous delivery robot built on a Raspberry Pi 5 with ROS2 Jazzy. Uses monocular VSLAM + ArUco markers + IMU fusion for localization, Nav2 for path planning, mecanum drive for omnidirectional movement, and a web UI for manual/autonomous control.

## Hardware

| Component | Details |
|---|---|
| Computer | Raspberry Pi 5 (8 GB), Ubuntu 24.04, ROS2 Jazzy |
| Camera | Pi Camera Module 3 (IMX708, monocular) |
| Motor drivers | 2x L298N dual H-bridge boards |
| Drive | 4x DC motors with mecanum wheels |
| IMU | MPU6050 via I2C (6-axis: accel + gyro) |

See [HARDWARE.md](HARDWARE.md) for full GPIO pinout, wiring diagrams, and power connections.

## Architecture

```mermaid
graph LR
  subgraph sensors [Sensors]
    Cam["Pi Camera Module 3"]
    IMU["MPU6050 IMU"]
  end

  subgraph perception [Perception]
    VSLAM["ORB-SLAM3 Node"]
    Aruco["ArUco Detector"]
  end

  subgraph localization [Localization]
    EKF["robot_localization EKF"]
  end

  subgraph nav [Navigation -- Phase 3]
    Nav2["Nav2 Stack"]
  end

  subgraph control [Control]
    WebUI["Web UI Node"]
    MotorDrv["Motor Driver Node"]
  end

  subgraph actuators [Actuators]
    L298N["2x L298N"]
    Wheels["4x Mecanum"]
  end

  Cam --> VSLAM -->|"/visual_odom"| EKF
  Cam --> Aruco -->|"/aruco/pose"| EKF
  IMU -->|"/imu/data_raw"| EKF
  EKF -->|"/odometry/filtered"| Nav2
  EKF -->|"/odometry/filtered"| WebUI
  Nav2 -->|"/cmd_vel"| MotorDrv
  WebUI -->|"/cmd_vel"| MotorDrv
  MotorDrv --> L298N --> Wheels
  Cam --> WebUI
  VSLAM -->|"/vslam/map_points"| WebUI
  Aruco -->|"/aruco/detections"| WebUI
```

## Package Layout

```
ros2_ws/src/delivery_robot/
  delivery_robot_bringup/    # launch files, config YAML
  delivery_robot_msgs/       # custom messages (ament_cmake)
  pi_camera_driver/          # rpicam-vid camera node (ament_python)
  motor_driver/              # L298N + mecanum kinematics (ament_python)
  imu_driver/                # MPU6050 IMU driver (ament_python)
  robot_web_ui/              # web dashboard + WebSocket bridge (ament_python)
  aruco_detector/            # ArUco marker detection + localization (ament_python)
  orb_slam3_ros/             # ORB-SLAM3 monocular wrapper (ament_cmake)
```

| Package | Main source | Description |
|---|---|---|
| `pi_camera_driver` | `pi_camera_driver/camera_node.py` | Launches `rpicam-vid` in a clean env, publishes JPEG and raw Image topics |
| `motor_driver` | `motor_driver/motor_driver_node.py` | Subscribes to `/cmd_vel`, mecanum inverse kinematics, L298N PWM via `lgpio` |
| `robot_web_ui` | `robot_web_ui/web_ui_node.py` | aiohttp server on port 8080: MJPEG stream, WebSocket bridge, localization UI, calibration |
| `delivery_robot_msgs` | `msg/MotorStatus.msg`, `msg/ArucoDetections.msg` | Motor status + ArUco detection messages |
| `delivery_robot_bringup` | `launch/bringup.launch.py` | Launches all base nodes with YAML config |
| `aruco_detector` | `aruco_detector/aruco_detector_node.py` | Detects ArUco markers, publishes pose + pixel detections for EKF and UI |
| `imu_driver` | `imu_driver/imu_driver_node.py` | MPU6050 I2C driver, publishes `sensor_msgs/Imu` at 50 Hz, startup calibration |
| `orb_slam3_ros` | `src/orb_slam3_node.cpp` | ORB-SLAM3 monocular VSLAM, publishes odometry + point cloud |

## ROS2 Topics

### Base System (Phase 1)

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | pi_camera_driver | orb_slam3, aruco_detector |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | pi_camera_driver | robot_web_ui |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | pi_camera_driver | aruco_detector |
| `/cmd_vel` | `geometry_msgs/Twist` | robot_web_ui | motor_driver |
| `/motor_status` | `MotorStatus` | motor_driver | robot_web_ui |

### Localization Stack (Phase 2)

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/visual_odom` | `nav_msgs/Odometry` | orb_slam3_ros | EKF, robot_web_ui |
| `/vslam/map_points` | `sensor_msgs/PointCloud2` | orb_slam3_ros | robot_web_ui |
| `/aruco/pose` | `PoseWithCovarianceStamped` | aruco_detector | EKF |
| `/aruco/detections` | `ArucoDetections` | aruco_detector | robot_web_ui |
| `/imu/data_raw` | `sensor_msgs/Imu` | imu_driver | EKF |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF | robot_web_ui, (Nav2) |

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Incremental rebuild of a single package:

```bash
colcon build --packages-select <package_name>
```

## Run

```bash
# Base system (camera, motors, web UI)
cd ~/ros2_ws && ./start_robot.sh

# Localization stack (in a separate terminal)
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch delivery_robot_bringup localization.launch.py

# Localization without ArUco (VSLAM only)
ros2 launch delivery_robot_bringup localization.launch.py use_aruco:=false
```

Open `http://<pi-ip>:8080` on your phone or laptop.

## Web UI

### Controls

| Input | Action |
|---|---|
| **W/A/S/D** | Forward / strafe left / back / strafe right |
| **Q/E** | Rotate left / right |
| **Shift** | Boost to full speed |
| **Speed slider** | Adjust max velocity (10-100%) |
| **Touch joystick** | Drag for omnidirectional control |
| **Gamepad left stick** | Translation (forward/back + strafe) |
| **Gamepad right stick** | Rotation |
| **Gamepad RT** | Boost |

### Localization Dashboard

The web UI includes five localization panels (visible at `http://<pi-ip>:8080`):

| Panel | What it shows |
|---|---|
| **Localization Status** | VSLAM state (initializing/tracking/lost), EKF active indicator, robot position (x, y, yaw), number of visible ArUco markers |
| **2D Map** | Top-down canvas with marker positions, robot location as a directional triangle, and a breadcrumb trail |
| **3D Point Cloud** | Interactive three.js viewer showing the sparse VSLAM map (rotate/zoom/pan) |
| **ArUco Overlay** | Green quadrilaterals drawn over detected markers on the camera feed |
| **Camera Calibration** | Built-in checkerboard calibration with live corner detection, frame counter, and RMS error report |

The camera feed also has a fullscreen button (top-right corner).

## Configuration

All config lives in `delivery_robot_bringup/config/`:

| File | Contents |
|---|---|
| `motor_pins.yaml` | GPIO pin map, PWM frequency, `min_duty`, `max_speed`, watchdog timeout, gyro correction PID |
| `camera.yaml` | Resolution, framerate, JPEG quality, `publish_raw` toggle, calibration file path |
| `camera_calibration.yaml` | Camera intrinsics (K, D, R, P matrices) |
| `aruco_markers.yaml` | Marker size, dictionary, known marker world positions |
| `imu.yaml` | MPU6050 config: I2C bus/address, publish rate, calibration offsets |
| `ekf.yaml` | robot_localization EKF config: sensor sources, covariances |
| `orb_slam3_pi5.yaml` | ORB-SLAM3 camera params, feature count, scale levels |

Edit and rebuild `delivery_robot_bringup` to apply changes.

## Known Quirks

- **libcamera/PiSP**: ROS2's `LD_LIBRARY_PATH` shadows the system libcamera. The camera node launches `rpicam-vid` in a minimal environment that strips all ROS2 paths. Do not change this without testing.
- **gpiochip4**: Pi 5 uses `gpiochip4` (RP1) for the 40-pin header, not `gpiochip0`. Set via `gpio_chip` param.
- **GPIO permissions**: The user must be in the `dialout` group. The launch script uses `sg dialout`.
- **I2C permissions**: The user must be in the `i2c` group for the IMU. Run `sudo usermod -aG i2c pi` and re-login.
- **ROS2 fixed-size arrays**: `float32[4]` and `bool[4]` fields must be assigned element-by-element in Python, not via list assignment.
- **JSON serialization**: ROS2 `float32`/`bool` types are numpy types. Convert with `float()`/`bool()` before passing to `json.dumps`.
- **OpenCV 4.6 ArUco API**: System OpenCV uses the older `cv2.aruco.detectMarkers()` function. Do NOT use the newer `ArucoDetector` class (4.7+).
- **ORB-SLAM3 memory**: Loads ~140MB vocabulary. 4GB swap is configured. Reduce `nFeatures` in config if OOM occurs.

## Further Reading

- [HARDWARE.md](HARDWARE.md) -- GPIO pinout, wiring, mecanum kinematics
- [ROADMAP.md](ROADMAP.md) -- phased development plan and open decisions
- [LOCALIZATION.md](LOCALIZATION.md) -- localization system user guide and developer reference
