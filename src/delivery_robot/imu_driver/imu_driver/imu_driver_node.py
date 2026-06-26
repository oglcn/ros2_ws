#!/usr/bin/env python3
"""
MPU6050 IMU driver node.

Reads accelerometer and gyroscope data over I2C from an MPU6050 and publishes
sensor_msgs/Imu messages. Supports startup calibration (gyro bias removal)
and configurable offsets loaded from a calibration file.
"""

import math
import time
import struct

import numpy as np
from smbus2 import SMBus

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


# MPU6050 register map
_REG_PWR_MGMT_1 = 0x6B
_REG_PWR_MGMT_2 = 0x6C
_REG_SMPLRT_DIV = 0x19
_REG_CONFIG = 0x1A
_REG_GYRO_CONFIG = 0x1B
_REG_ACCEL_CONFIG = 0x1C
_REG_ACCEL_XOUT_H = 0x3B
_REG_TEMP_OUT_H = 0x41
_REG_GYRO_XOUT_H = 0x43
_REG_WHO_AM_I = 0x75

# MPU6050 = 0x68, MPU6500/ICM-20689 = 0x70 (register-compatible variants)
_WHO_AM_I_VALID = {0x68, 0x70}
_WHO_AM_I_NAMES = {0x68: 'MPU6050', 0x70: 'MPU6500'}

# Conversion factors for ±2g accel and ±250°/s gyro (defaults)
_ACCEL_SCALE = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}   # LSB/g
_GYRO_SCALE = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}             # LSB/(°/s)

_GRAVITY = 9.80665


class Mpu6050Driver:
    """Low-level MPU6050 I2C driver."""

    def __init__(self, bus_num, address, accel_range=0, gyro_range=0, dlpf_bw=3):
        self._bus_num = bus_num
        self._addr = address
        self._accel_range = accel_range
        self._gyro_range = gyro_range
        self._dlpf_bw = dlpf_bw
        self._accel_lsb = _ACCEL_SCALE[accel_range]
        self._gyro_lsb = _GYRO_SCALE[gyro_range]
        self._bus = None
        self.chip_name = 'unknown'

        self._init_hw()

    def _init_hw(self, hard_reset=True):
        """Initialize or re-initialize the I2C connection and sensor registers.

        Args:
            hard_reset: If True, send device reset (0x80). Use False for
                        soft reconnect to avoid disrupting the I2C bus.
        """
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass

        self._bus = SMBus(self._bus_num)

        if hard_reset:
            self._bus.write_byte_data(self._addr, _REG_PWR_MGMT_1, 0x80)
            time.sleep(0.1)

        who = self._bus.read_byte_data(self._addr, _REG_WHO_AM_I)
        if who not in _WHO_AM_I_VALID:
            raise RuntimeError(
                f'IMU WHO_AM_I mismatch: got 0x{who:02x}, '
                f'expected one of {[f"0x{v:02x}" for v in _WHO_AM_I_VALID]}'
            )
        self.chip_name = _WHO_AM_I_NAMES.get(who, f'unknown(0x{who:02x})')

        self._bus.write_byte_data(self._addr, _REG_PWR_MGMT_1, 0x01)
        time.sleep(0.01)
        self._bus.write_byte_data(self._addr, _REG_PWR_MGMT_2, 0x00)

        self._bus.write_byte_data(self._addr, _REG_CONFIG, self._dlpf_bw & 0x07)
        self._bus.write_byte_data(self._addr, _REG_SMPLRT_DIV, 0x00)

        self._bus.write_byte_data(
            self._addr, _REG_ACCEL_CONFIG, (self._accel_range & 0x03) << 3)
        self._bus.write_byte_data(
            self._addr, _REG_GYRO_CONFIG, (self._gyro_range & 0x03) << 3)
        time.sleep(0.01)

    def read_raw(self):
        """Read all 14 bytes (accel[6] + temp[2] + gyro[6]) in one burst."""
        data = self._bus.read_i2c_block_data(self._addr, _REG_ACCEL_XOUT_H, 14)
        vals = struct.unpack('>hhhhhhh', bytes(data))
        # vals: ax, ay, az, temp, gx, gy, gz -- skip temp at index 3
        return vals[0], vals[1], vals[2], vals[4], vals[5], vals[6]

    def read_si(self):
        """Read accelerometer (m/s²) and gyroscope (rad/s) in SI units."""
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.read_raw()
        ax = (ax_raw / self._accel_lsb) * _GRAVITY
        ay = (ay_raw / self._accel_lsb) * _GRAVITY
        az = (az_raw / self._accel_lsb) * _GRAVITY
        gx = math.radians(gx_raw / self._gyro_lsb)
        gy = math.radians(gy_raw / self._gyro_lsb)
        gz = math.radians(gz_raw / self._gyro_lsb)
        return ax, ay, az, gx, gy, gz

    def close(self):
        self._bus.close()


class ImuDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('accel_range', 0)      # 0=±2g, 1=±4g, 2=±8g, 3=±16g
        self.declare_parameter('gyro_range', 0)        # 0=±250°/s, 1=±500, 2=±1000, 3=±2000
        self.declare_parameter('dlpf_bandwidth', 3)    # DLPF setting (see Mpu6050Driver)
        self.declare_parameter('calibrate_samples', 500)

        # Calibration offsets (loaded from config or set by calibrate_imu tool)
        self.declare_parameter('gyro_offset_x', 0.0)
        self.declare_parameter('gyro_offset_y', 0.0)
        self.declare_parameter('gyro_offset_z', 0.0)
        self.declare_parameter('accel_offset_x', 0.0)
        self.declare_parameter('accel_offset_y', 0.0)
        self.declare_parameter('accel_offset_z', 0.0)
        self.declare_parameter('heading_deadband', 0.015)

        bus = self.get_parameter('i2c_bus').value
        addr = self.get_parameter('i2c_address').value
        rate = self.get_parameter('publish_rate').value
        self._frame_id = self.get_parameter('frame_id').value
        accel_range = self.get_parameter('accel_range').value
        gyro_range = self.get_parameter('gyro_range').value
        dlpf_bw = self.get_parameter('dlpf_bandwidth').value
        cal_samples = self.get_parameter('calibrate_samples').value

        self._gyro_offset = np.array([
            self.get_parameter('gyro_offset_x').value,
            self.get_parameter('gyro_offset_y').value,
            self.get_parameter('gyro_offset_z').value,
        ])
        self._accel_offset = np.array([
            self.get_parameter('accel_offset_x').value,
            self.get_parameter('accel_offset_y').value,
            self.get_parameter('accel_offset_z').value,
        ])

        self._i2c_bus = bus
        self._i2c_addr = addr
        self._accel_range = accel_range
        self._gyro_range = gyro_range
        self._dlpf_bw = dlpf_bw
        self._consecutive_errors = 0
        self._max_errors_before_reinit = 50  # ~1 second of failures at 50Hz
        self._last_reinit_time = 0.0
        self._reinit_cooldown = 10.0  # seconds between reinit attempts

        self.get_logger().info(
            f'Opening IMU on I2C bus {bus}, address 0x{addr:02x}'
        )

        self._mpu = None
        self._try_init_imu()

        if self._mpu and cal_samples > 0 and np.allclose(self._gyro_offset, 0.0):
            self._run_startup_calibration(cal_samples)

        # Covariance values (diagonal) -- reasonable defaults for MPU6050
        # Gyro noise density: ~0.005 rad/s/√Hz → variance ≈ 0.0001 rad²/s²
        # Accel noise density: ~0.004 m/s²/√Hz → variance ≈ 0.01 m²/s⁴
        self._gyro_cov = [0.0001, 0.0, 0.0,
                          0.0, 0.0001, 0.0,
                          0.0, 0.0, 0.0001]
        self._accel_cov = [0.01, 0.0, 0.0,
                           0.0, 0.01, 0.0,
                           0.0, 0.0, 0.01]
        # Orientation not provided by this sensor
        self._orient_cov = [-1.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0]

        self._imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self._heading_pub = self.create_publisher(Float64, 'imu/heading', 10)

        self._heading_deg = 0.0
        self._heading_deadband = self.get_parameter('heading_deadband').value
        self._publish_dt = 1.0 / rate

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'IMU publishing at {rate} Hz on /{self._imu_pub.topic_name}'
        )

    def _try_init_imu(self):
        """Attempt to initialize the IMU, retrying up to 5 times."""
        for attempt in range(5):
            try:
                self._mpu = Mpu6050Driver(
                    self._i2c_bus, self._i2c_addr,
                    self._accel_range, self._gyro_range, self._dlpf_bw,
                )
                self._consecutive_errors = 0
                self.get_logger().info(
                    f'{self._mpu.chip_name} initialized successfully'
                )
                return
            except Exception as e:
                self.get_logger().warning(
                    f'IMU init attempt {attempt + 1}/5 failed: {e}'
                )
                time.sleep(0.5)
        self.get_logger().error(
            'IMU initialization failed after 5 attempts, will retry on publish'
        )
        self._mpu = None

    def _reinit_imu(self):
        """Soft re-initialize after repeated I2C errors, with cooldown.

        Closes and re-opens the I2C bus without sending a hard reset to the
        sensor, which avoids disrupting the bus further.
        """
        now = time.time()
        if now - self._last_reinit_time < self._reinit_cooldown:
            return
        self._last_reinit_time = now
        self._consecutive_errors = 0

        self.get_logger().warning('Too many I2C errors, reconnecting IMU (soft)...')
        if self._mpu is not None:
            try:
                self._mpu.close()
            except Exception:
                pass
            self._mpu = None

        time.sleep(0.5)

        try:
            self._mpu = Mpu6050Driver.__new__(Mpu6050Driver)
            self._mpu._bus_num = self._i2c_bus
            self._mpu._addr = self._i2c_addr
            self._mpu._accel_range = self._accel_range
            self._mpu._gyro_range = self._gyro_range
            self._mpu._dlpf_bw = self._dlpf_bw
            self._mpu._accel_lsb = _ACCEL_SCALE[self._accel_range]
            self._mpu._gyro_lsb = _GYRO_SCALE[self._gyro_range]
            self._mpu._bus = None
            self._mpu.chip_name = 'unknown'
            self._mpu._init_hw(hard_reset=False)
            self.get_logger().info(f'{self._mpu.chip_name} reconnected (soft)')
        except Exception as e:
            self._mpu = None
            self.get_logger().warning(f'IMU reconnect failed: {e}')

    def _run_startup_calibration(self, n_samples):
        """Collect samples while stationary to estimate gyro bias."""
        self.get_logger().info(
            f'Calibrating gyro bias ({n_samples} samples) -- keep the robot still...'
        )
        gyro_sum = np.zeros(3)
        accel_sum = np.zeros(3)
        good = 0
        for _ in range(n_samples + 50):
            try:
                ax, ay, az, gx, gy, gz = self._mpu.read_si()
            except OSError:
                continue
            # Skip first 50 samples to let sensor settle
            if good < 50:
                good += 1
                time.sleep(0.002)
                continue
            gyro_sum += [gx, gy, gz]
            accel_sum += [ax, ay, az]
            good += 1
            time.sleep(0.002)

        count = good - 50
        if count > 0:
            self._gyro_offset = gyro_sum / count
            mean_accel = accel_sum / count
            # Accel offset: remove gravity from Z (assumes board is level, Z up)
            self._accel_offset = np.array([
                mean_accel[0],
                mean_accel[1],
                mean_accel[2] - _GRAVITY,
            ])
            self.get_logger().info(
                f'Calibration done: gyro_offset=({self._gyro_offset[0]:.5f}, '
                f'{self._gyro_offset[1]:.5f}, {self._gyro_offset[2]:.5f}) rad/s, '
                f'accel_offset=({self._accel_offset[0]:.4f}, '
                f'{self._accel_offset[1]:.4f}, {self._accel_offset[2]:.4f}) m/s²'
            )
        else:
            self.get_logger().warning('Calibration failed: no valid samples')

    def _publish(self):
        if self._mpu is None:
            self._reinit_imu()
            return

        try:
            ax, ay, az, gx, gy, gz = self._mpu.read_si()
        except OSError as e:
            self._consecutive_errors += 1
            if self._consecutive_errors >= self._max_errors_before_reinit:
                self._reinit_imu()
            else:
                self.get_logger().warning(
                    f'I2C read error ({self._consecutive_errors}): {e}',
                    throttle_duration_sec=5.0,
                )
            return

        self._consecutive_errors = 0

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.angular_velocity.x = gx - self._gyro_offset[0]
        msg.angular_velocity.y = gy - self._gyro_offset[1]
        msg.angular_velocity.z = gz - self._gyro_offset[2]

        msg.linear_acceleration.x = ax - self._accel_offset[0]
        msg.linear_acceleration.y = ay - self._accel_offset[1]
        msg.linear_acceleration.z = az - self._accel_offset[2]

        msg.orientation_covariance = self._orient_cov
        msg.angular_velocity_covariance = self._gyro_cov
        msg.linear_acceleration_covariance = self._accel_cov

        self._imu_pub.publish(msg)

        # Drift-corrected heading: integrate gyro Z with deadband
        gz_corrected = msg.angular_velocity.z
        if abs(gz_corrected) > self._heading_deadband:
            self._heading_deg += math.degrees(gz_corrected) * self._publish_dt
            self._heading_deg %= 360.0
        self._heading_pub.publish(Float64(data=self._heading_deg))

    def destroy_node(self):
        if hasattr(self, '_mpu'):
            self._mpu.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
