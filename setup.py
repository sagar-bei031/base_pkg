from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'base_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sagar',
    maintainer_email='078bei031.sagar@pcampus.edu.np',
    description='Base Link to ROS',
    license='null',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odome_node = base_pkg.odom_node:main',
            'joy_node = base_pkg.joy_node:main',
            'cmd_node = base_pkg.cmd_node:main',
            'imu_node = base_pkg.imu_node:main',
            'odom_sub_node = base_pkg.odom_subscriber:main',
            'imu_sub_node = base_pkg.imu_subscriber:main',
            'odom_cmd_serial_node = base_pkg.odom_cmd_serial:main',
        ],
    },
)
