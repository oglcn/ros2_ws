# Delivery Robot -- Hardware & Wiring Reference

## Platform

| Component | Details |
|---|---|
| Computer | Raspberry Pi 5 (8 GB), Ubuntu 24.04 |
| Camera | Pi Camera Module 3 (IMX708, monocular) |
| Motor drivers | 2x L298N dual H-bridge boards |
| Motors | 4x DC motors with mecanum wheels |
| IMU | MPU6050 (I2C, 6-axis: accel + gyro) |

## GPIO Pinout (Pi 5, gpiochip4 / RP1)

Each motor uses 3 GPIO pins: **EN** (PWM speed), **IN1** and **IN2** (direction).

### Motor-to-GPIO Map

| Motor | EN (PWM) | IN1 | IN2 | L298N Board |
|---|---|---|---|---|
| Front Left | GPIO 23 | GPIO 25 | GPIO 24 | Board 2, Channel B |
| Front Right | GPIO 18 | GPIO 21 | GPIO 20 | Board 2, Channel A |
| Rear Left | GPIO 13 | GPIO 19 | GPIO 16 | Board 1, Channel B |
| Rear Right | GPIO 12 | GPIO 6 | GPIO 5 | Board 1, Channel A |

### GPIO-to-Physical Pin Map

| GPIO | Physical Pin | Function |
|---|---|---|
| GPIO 5 | Pin 29 | Rear Right IN2 |
| GPIO 6 | Pin 31 | Rear Right IN1 |
| GPIO 12 | Pin 32 | Rear Right EN (PWM) |
| GPIO 13 | Pin 33 | Rear Left EN (PWM) |
| GPIO 16 | Pin 36 | Rear Left IN2 |
| GPIO 18 | Pin 12 | Front Right EN (PWM) |
| GPIO 19 | Pin 35 | Rear Left IN1 |
| GPIO 20 | Pin 38 | Front Right IN2 |
| GPIO 21 | Pin 40 | Front Right IN1 |
| GPIO 23 | Pin 16 | Front Left EN (PWM) |
| GPIO 24 | Pin 18 | Front Left IN2 |
| GPIO 25 | Pin 22 | Front Left IN1 |

### Power Connections

| Connection | From | To |
|---|---|---|
| Motor power (VM) | Battery (6-12V) | L298N VM terminal |
| Logic power (5V) | L298N 5V output **or** Pi 5V (Pin 2/4) | L298N VCC |
| Common ground | Pi GND (Pin 6/9/14/20/25/30/34/39) | L298N GND |

> **Important**: The Pi 5 and both L298N boards must share a common ground.
> If your motors draw heavy current, power the L298N VM from a separate battery
> and only share GND with the Pi -- do not feed motor power through the Pi.

### Direction Logic (per motor)

| IN1 | IN2 | Motor Action |
|---|---|---|
| HIGH | LOW | Forward |
| LOW | HIGH | Reverse |
| LOW | LOW | Coast (stop) |

EN pin PWM duty cycle controls speed (0-100%).

## Mecanum Wheel Layout

```
    FRONT
  ┌───┐ ┌───┐
  │ FL│ │FR │
  │ / │ │ \ │     Arrow shows roller angle
  └───┘ └───┘
  ┌───┐ ┌───┐
  │RL │ │ RR│
  │ \ │ │ / │
  └───┘ └───┘
    REAR
```

### Inverse Kinematics

```
FL = vx - vy - ω
FR = vx + vy + ω
RL = vx + vy - ω
RR = vx - vy + ω
```

Where `vx` = forward, `vy` = strafe left, `ω` = counter-clockwise rotation.

### Gyro Yaw-Rate Correction

The motor driver subscribes to `/imu/data_raw` and applies a PID correction to the `ω` term in the kinematics above. This counteracts rotational drift caused by unequal motor characteristics or uneven weight distribution. A high-pass filter on the gyro signal rejects slow sensor drift while passing through real rotation errors.

Configuration in `motor_pins.yaml` under `gyro_correction`:

| Parameter | Default | Description |
|---|---|---|
| `enabled` | `true` | Master switch |
| `kp` | `0.5` | Proportional gain |
| `ki` | `0.1` | Integral gain (fixes persistent bias) |
| `kd` | `0.0` | Derivative gain (normally not needed) |
| `i_max` | `0.3` | Integral windup clamp |
| `highpass_rc` | `2.0` | High-pass time constant in seconds (drift rejection) |
| `deadband` | `0.01` | Ignore gyro readings below this (rad/s) |

**Tuning:** Start with only P (set `ki: 0.0`). Increase `kp` until the robot holds a straight line. If it oscillates (wobbles), reduce `kp`. Then add small `ki` (0.05-0.2) to eliminate steady-state drift. The correction auto-resets when the robot stops.

## Camera

- Connected via Pi CSI ribbon cable
- Uses `rpicam-vid` (from PPA `ppa:manajev/pi5-camera`) for PiSP support
- Launched in a clean environment (no ROS2 `LD_LIBRARY_PATH`) to avoid libcamera version conflicts

## ROS2 Topics (Phase 1)

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | pi_camera_driver | (Phase 2: VSLAM, ArUco) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | pi_camera_driver | robot_web_ui |
| `/cmd_vel` | `geometry_msgs/Twist` | robot_web_ui | motor_driver |
| `/motor_status` | `delivery_robot_msgs/MotorStatus` | motor_driver | robot_web_ui |

## Configuration Files

All config lives in `delivery_robot_bringup/config/`:

- **`motor_pins.yaml`** -- GPIO pin assignments, PWM frequency, speed limits, watchdog timeout
- **`camera.yaml`** -- resolution and framerate

Edit these files and rebuild (`colcon build --packages-select delivery_robot_bringup`) to apply changes.

## Quick Start

```bash
~/start_robot.sh
```

Then open `http://10.171.31.166:8080` on your phone or laptop.

- **WASD** -- drive forward/back/strafe
- **Q/E** -- rotate left/right
- **Shift** -- boost to full speed
- **Speed slider** -- adjust max velocity (10-100%)
- **Joystick** -- touch/mouse drag for omnidirectional control

Ctrl+C in the terminal to stop all nodes.

## IMU (MPU6050)

| Property | Value |
|---|---|
| Sensor | MPU6050 (6-axis: 3-axis accelerometer + 3-axis gyroscope) |
| Interface | I2C bus 1 (SDA=GPIO 2/Pin 3, SCL=GPIO 3/Pin 5) |
| Address | 0x68 (default) |
| Required group | `i2c` (user `pi` must be in this group) |

**Wiring:**

| MPU6050 Pin | Pi 5 Pin | Notes |
|---|---|---|
| VCC | Pin 1 (3.3V) | 3.3V supply (do NOT use 5V) |
| GND | Pin 6 (GND) | Common ground with Pi |
| SDA | Pin 3 (GPIO 2) | I2C data |
| SCL | Pin 5 (GPIO 3) | I2C clock |
| AD0 | GND or unconnected | Address bit: GND=0x68, VCC=0x69 |
| INT | (unused) | Interrupt pin, not connected |

**Mounting:** Mount flat on the robot chassis, aligned with `base_link` axes (X forward, Y left, Z up). The static TF `base_link -> imu_link` is set to (0, 0, 0.05m) in the launch file -- adjust if your mounting differs.

## ArUco Markers

| Property | Value |
|---|---|
| Type | Printed ArUco markers (DICT_4X4_50) |
| Size | 15cm x 15cm |
| Interface | Camera (visual detection) |
| Purpose | Absolute position fixes for EKF |

See [ROADMAP.md](ROADMAP.md) for the full development plan.
