from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rs485_main_ros2'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='ROS2 RS485 base driver: differential drive encoder odometry via CRC-16/Modbus',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver_node = rs485_main_ros2.base_driver_node:main',
        ],
    },
)
