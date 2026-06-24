from setuptools import find_packages, setup

package_name = 'pi_camera_driver'

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
    description='Pi Camera Module 3 driver using rpicam-vid for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = pi_camera_driver.camera_node:main',
            'calibrate_camera = pi_camera_driver.calibrate_camera:main',
        ],
    },
)
