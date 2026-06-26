#!/usr/bin/env python3
"""
Offline IMU calibration tool.

Collects samples from a stationary MPU6050, computes gyro bias and
accelerometer offset, and writes a YAML config file that the imu_driver_node
loads at startup to skip live calibration.

Usage:
  ros2 run imu_driver calibrate_imu
  ros2 run imu_driver calibrate_imu -- --samples 1000 --bus 1 --output /tmp/imu_cal.yaml
"""

import argparse
import os
import sys
import time

import numpy as np
import yaml

# Reuse the low-level driver
from imu_driver.imu_driver_node import Mpu6050Driver, _GRAVITY


def main(args=None):
    parser = argparse.ArgumentParser(description='Calibrate MPU6050 IMU')
    parser.add_argument('--bus', type=int, default=1, help='I2C bus number')
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x68,
                        help='I2C address (hex)')
    parser.add_argument('--samples', type=int, default=1000,
                        help='Number of samples to collect')
    parser.add_argument('--output', type=str,
                        default=os.path.expanduser(
                            '~/ros2_ws/src/delivery_robot/delivery_robot_bringup'
                            '/config/imu_calibration.yaml'),
                        help='Output YAML file path')

    parsed = parser.parse_args(args=sys.argv[1:] if args is None else [])

    print(f'Opening MPU6050 on I2C bus {parsed.bus}, address 0x{parsed.address:02x}')
    mpu = Mpu6050Driver(parsed.bus, parsed.address)

    print(f'Collecting {parsed.samples} samples -- keep the robot STILL and LEVEL...')
    time.sleep(1.0)

    # Warm-up
    for _ in range(100):
        mpu.read_si()
        time.sleep(0.002)

    gyro_samples = []
    accel_samples = []
    for i in range(parsed.samples):
        ax, ay, az, gx, gy, gz = mpu.read_si()
        accel_samples.append([ax, ay, az])
        gyro_samples.append([gx, gy, gz])
        time.sleep(0.002)
        if (i + 1) % 100 == 0:
            print(f'  {i + 1}/{parsed.samples}')

    mpu.close()

    gyro = np.array(gyro_samples)
    accel = np.array(accel_samples)

    gyro_mean = gyro.mean(axis=0)
    gyro_std = gyro.std(axis=0)
    accel_mean = accel.mean(axis=0)
    accel_std = accel.std(axis=0)

    accel_offset = accel_mean.copy()
    accel_offset[2] -= _GRAVITY

    print()
    print('=== Calibration Results ===')
    print(f'Gyro bias (rad/s):   x={gyro_mean[0]:.6f}  y={gyro_mean[1]:.6f}  z={gyro_mean[2]:.6f}')
    print(f'Gyro std  (rad/s):   x={gyro_std[0]:.6f}  y={gyro_std[1]:.6f}  z={gyro_std[2]:.6f}')
    print(f'Accel mean (m/s²):   x={accel_mean[0]:.4f}  y={accel_mean[1]:.4f}  z={accel_mean[2]:.4f}')
    print(f'Accel std  (m/s²):   x={accel_std[0]:.4f}  y={accel_std[1]:.4f}  z={accel_std[2]:.4f}')
    print(f'Accel offset (m/s²): x={accel_offset[0]:.4f}  y={accel_offset[1]:.4f}  z={accel_offset[2]:.4f}')
    print(f'Gravity magnitude:   {np.linalg.norm(accel_mean):.4f} m/s² (expected {_GRAVITY:.4f})')

    cal_data = {
        'imu_driver': {
            'ros__parameters': {
                'gyro_offset_x': float(gyro_mean[0]),
                'gyro_offset_y': float(gyro_mean[1]),
                'gyro_offset_z': float(gyro_mean[2]),
                'accel_offset_x': float(accel_offset[0]),
                'accel_offset_y': float(accel_offset[1]),
                'accel_offset_z': float(accel_offset[2]),
            }
        }
    }

    os.makedirs(os.path.dirname(parsed.output), exist_ok=True)
    with open(parsed.output, 'w') as f:
        f.write('# MPU6050 calibration offsets\n')
        f.write(f'# Generated: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
        f.write(f'# Samples: {parsed.samples}\n')
        f.write(f'# Gravity measured: {np.linalg.norm(accel_mean):.4f} m/s²\n\n')
        yaml.dump(cal_data, f, default_flow_style=False)

    print(f'\nCalibration saved to: {parsed.output}')
    print('Rebuild delivery_robot_bringup and restart to apply.')


if __name__ == '__main__':
    main()
