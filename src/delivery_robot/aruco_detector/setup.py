from setuptools import find_packages, setup

package_name = 'aruco_detector'

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
    description='ArUco marker detection and robot localization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_detector_node = aruco_detector.aruco_detector_node:main',
        ],
    },
)
