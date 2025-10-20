from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab5_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for every ROS 2 Python package
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Add this line for launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # (optional) include config or maps if you want them installed too
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patrick',
    maintainer_email='patrickyan51418@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
