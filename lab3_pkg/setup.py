from setuptools import find_packages, setup

package_name = 'lab3_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lameya',
    maintainer_email='Lameyadia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'odom_logger = lab3_pkg.odom_logger:main',
        	'plot_odom = lab3_pkg.plot_odom:main',
		'compare_plot = lab3_pkg.compare_plot:main',
        ],
    },
)
