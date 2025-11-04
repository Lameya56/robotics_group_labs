from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lab5_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),   glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=name,
    maintainer_email='email',
    description='Lab 5: SLAM + Nav2 (TurtleBot3 only)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_aware_waypoint_nav = lab5_nav.obstacle_aware_waypoint_nav:main',
        ],
    },
)
