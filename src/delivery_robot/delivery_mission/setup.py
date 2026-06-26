from setuptools import find_packages, setup

package_name = 'delivery_mission'

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
    description='Delivery mission manager with waypoint navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager_node = delivery_mission.mission_manager_node:main',
        ],
    },
)
