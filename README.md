## Part 1
**Step 1 - Run the Odom Logger**

In a terminal, run the following:
- cd ~/ros2_ws
- colcon build
- source install/setup.bash
- ros2 run lab3_pkg odom_logger

**Step 2 - Run Gazebo & RVizz2**

In a separate terminal from Step 1:
- cd ~/ros2_ws
- colcon build --symlink-install
- source install/setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

In another terminal, launch RViz2 so you can visualize /odom and the robot frames throughout the run:
- cd ~/ros2_ws
- colcon build 
- source install/setup.bash
- ros2 launch turtlebot3_bringup rviz2.launch.py

In a third terminal, run square.py from lab 2
- cd ~/ros2_ws
- colcon build 
- source install/setup.bash
- ros2 run lab2_pkg square

**Step 3 - Generating the Graph**

In another terminal, generate the graph
- cd ~/ros2_ws
- colcon build 
- source install/setup.bash
- ros2 run lab3_pkg plot_odom

## Part 2

**Step 1 - Run Gazebo & RVizz2**

In a terminal, run the following::
- cd ~/ros2_ws
- colcon build --symlink-install
- source install/setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

In another terminal, launch RViz2 so you can visualize /odom and the robot frames throughout the run:
- cd ~/ros2_ws
- colcon build 
- source install/setup.bash
- ros2 launch turtlebot3_bringup rviz2.launch.py

**Step 2 - Running the PD Controller**

In another terminal, run the PD Controller by
- cd ~/ros2_ws
- colcon build
- source install/setup.bash
- ros2 run lab3_pkg pd_controller

## Part 3
**Step 1 - Setting Up Terminals**

Open three(3) separate terminals and run the following commands on each terminal:
- cd ~/ros2_ws
- colcon build
- source install/setup.bash

**Step 2 - Run Gazebo**

On the first terminal, run the following:
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

**Step 3 - Run RViz2**

On the second terminal, run the following:
- ros2 launch turtlebot3_bringup rviz2.launch.py

**Step 4 - Run PiD Control**

On the third terminal, to make the burger bot move along the left wall:
- ros2 run lab3_pkg wall_following

## Part 4
**Step 1 - Setting up Topic**

In another terminal, run the following:
- cd ~/ros2_ws
- source /opt/ros/jazzy/setup.bash
- source install/setup.bash
- colcon build
- ros2 launch lab3_pkg ekf_localization.launch.py

**Step 2 - Activating Logger**

In another terminal, run the following:
- cd ~/ros2_ws
- colcon build
- source install/setup.bash
- ros2 run lab3_pkg compare_logger

**Step 3 - Running the PD Controller**

In another terminal, run the PD Controller by
- cd ~/ros2_ws
- colcon build
- source install/setup.bash
- ros2 run lab3_pkg pd_controller

**Step 4 - Generate the Graph**

In another terminal, run the following:
- cd ~/ros2_ws
- colcon build
- source install/setup.bash
- ros2 run lab3_pkg compare_plot
