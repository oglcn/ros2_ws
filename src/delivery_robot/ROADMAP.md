# Delivery Robot -- Roadmap

## Phase 1: Manual Control -- DONE

All software for manual driving via web UI is complete and tested.

**What was built:**
- `pi_camera_driver` -- streams Pi Camera Module 3 at 640x480@30fps via `rpicam-vid`
- `motor_driver` -- mecanum inverse kinematics, L298N GPIO control via `lgpio`, safety watchdog
- `robot_web_ui` -- aiohttp web dashboard with MJPEG stream, virtual joystick, keyboard (WASD+QE), gamepad support, motor status bars
- `delivery_robot_msgs` -- `MotorStatus.msg`
- `delivery_robot_bringup` -- launch file + YAML config
- `aruco_detector` -- package stub for Phase 2

## Phase 1.5: Hardware Integration -- IN PROGRESS

Physical wiring of motors and L298N boards to the Pi 5 GPIO header.

**Tasks:**
- [ ] Wire 2x L298N boards to Pi 5 (12 GPIO pins + power + GND)
- [ ] Verify each motor spins in the correct direction
- [ ] Tune `min_duty` parameter (currently 0.3) for actual motor startup threshold
- [ ] Confirm mecanum wheel orientation matches kinematics (roller angles)
- [ ] Test omnidirectional movement: forward, strafe, rotate, diagonal
- [ ] Tune `max_speed` if needed for safe indoor operation

## Phase 2: Perception and Localization -- IN PROGRESS

Build the sensor fusion stack for autonomous positioning.

**ORB-SLAM3 (Monocular Visual SLAM):** -- DONE
- ORB-SLAM3 built from source on Pi 5 (no CUDA, OpenCV 4.6, 600 features, 6 levels)
- ROS2 C++ wrapper (`orb_slam3_ros`) publishes `/visual_odom` + TF `odom → base_link`
- Vocabulary loaded (~140MB), headless (no viewer)
- Library at `/home/pi/third_party/ORB_SLAM3/lib/libORB_SLAM3.so`

**ArUco Detector:** -- DONE
- `aruco_detector` node detects ArUco markers from `/camera/image_raw`
- Computes camera pose from known marker positions (`aruco_markers.yaml`)
- Publishes `PoseWithCovarianceStamped` to `/aruco/pose` for EKF fusion
- Broadcasts TF for detected markers
- Optional (launched via `use_aruco:=true/false`)

**Camera Enhancements:** -- DONE
- `publish_raw: true` enabled (raw BGR8 images for VSLAM + ArUco)
- `CameraInfo` publisher added (loads calibration from YAML)
- Calibration script: `ros2 run pi_camera_driver calibrate_camera`
- Default approximate calibration provided; run calibration for accuracy

**Sensor Fusion:** -- DONE
- `robot_localization` EKF configured (`ekf.yaml`)
- Fuses: VSLAM odometry (`/visual_odom`) + ArUco position fixes (`/aruco/pose`)
- Publishes `/odometry/filtered` and TF `map → odom`
- 2D mode for ground robots

**Static TF:** -- DONE
- `base_link → camera_link` static transform in bringup launch

**System Prep:** -- DONE
- 4GB swap file (persistent across reboots)
- CPU governor set to `performance`
- ORB-SLAM3 libraries in ldconfig

**Remaining:**
- [ ] Run camera calibration with checkerboard (accurate intrinsics)
- [ ] Test VSLAM tracking with live camera feed
- [ ] Print and place ArUco markers, measure positions
- [ ] Tune ORB-SLAM3 parameters based on real-world performance
- [ ] Add IMU driver when MPU6050 hardware arrives

**IMU Driver:**
- MPU6050 via I2C (SDA=GPIO 2, SCL=GPIO 3)
- Publish `sensor_msgs/Imu` to `/imu/data`
- Calibration routine for gyro bias and accelerometer offset
- *Blocked on*: IMU hardware arrival

## Phase 3: Autonomous Navigation -- FUTURE

Full autonomous delivery missions.

**Nav2 Integration:**
- Planner, controller, recovery behaviors
- Costmap from VSLAM-generated map
- Integration with `/odometry/filtered` from Phase 2

**Delivery Mission Manager:**
- Accept target ArUco marker as destination
- Plan route through known map
- Execute navigation with obstacle avoidance
- Report progress and status

**Web UI Autonomous Mode:**
- Enable the "Autonomous" mode button (currently greyed out)
- Set destination markers from the UI
- Monitor mission progress, pause/resume/cancel
- Manual override always available

## Open Decisions

| Decision | Options | Status |
|---|---|---|
| VSLAM package | ~~RTAB-Map vs stella_vslam~~ **ORB-SLAM3** | Decided -- built and working |
| Map representation | Occupancy grid vs point cloud | Depends on Nav2 integration |
| Wheel odometry | None planned (no encoders) | May add if VSLAM alone is insufficient |
| Battery monitoring | ADC via I2C or voltage divider | Not yet planned |
| VSLAM offloading | Run on-device vs remote | Starting on-device; offload-ready by design |
