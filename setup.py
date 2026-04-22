from setuptools import setup
from glob import glob
import os

package_name = 'line_tracking_avoidance'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Line tracking with obstacle avoidance - ROS 2 Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_detector     = line_tracking_avoidance.line_detector:main',
            'obstacle_detector = line_tracking_avoidance.obstacle_detector:main',
            'controller        = line_tracking_avoidance.controller:main',
        ],
    },
)
