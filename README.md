###How to build the package:  
cd ~/ros2_ws  
colcon build  
source install/setup.bash  

###How to run each node:

ros2 run lab3_pkg odom_logger  
ros2 run lab3_pkg pd_controller  
ros2 run lab3_pkg compare_plot  
