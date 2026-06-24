from setuptools import find_packages, setup

package_name = 'motor_driver'

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
    description='L298N motor driver with mecanum wheel kinematics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_driver_node = motor_driver.motor_driver_node:main',
        ],
    },
)
