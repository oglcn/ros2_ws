from setuptools import find_packages, setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='MPU6050 IMU driver over I2C for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_driver_node = imu_driver.imu_driver_node:main',
            'calibrate_imu = imu_driver.calibrate_imu:main',
        ],
    },
)
