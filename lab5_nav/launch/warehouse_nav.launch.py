import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    warehouse_pkg = os.path.expanduser('~/ros2_ws/src/aws-robomaker-small-warehouse-world')
    world = os.path.join(warehouse_pkg, 'worlds/small_warehouse.world')
    gz_resource = f"{warehouse_pkg}:{warehouse_pkg}/models"
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{os.environ.get('GZ_SIM_RESOURCE_PATH','')}:{gz_resource}"

    return LaunchDescription([
        # Launch the warehouse world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join('/opt/ros/jazzy/share/ros_gz_sim/launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': world}.items()
        ),
        # Optionally spawn TurtleBot3
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'tb3',
                '-file', os.path.join(
                    os.environ['COLCON_PREFIX_PATH'].split(':')[0],
                    'share/turtlebot3_description/urdf/turtlebot3_burger.urdf'),
                '-x', '10', '-y', '0', '-Y', '0'
            ],
            output='screen'
        )
    ])


