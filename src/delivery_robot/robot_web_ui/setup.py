import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_web_ui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aiohttp'],
    zip_safe=False,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Web dashboard for the delivery robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_ui_node = robot_web_ui.web_ui_node:main',
        ],
    },
)
