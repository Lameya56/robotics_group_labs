
#### To run Part 2:

tb3_world:  
MAP="$(ros2 pkg prefix lab5_nav)/share/lab5_nav/maps/tb3_world_map.yaml"  
ros2 launch lab5_nav nav_bringup.launch.py map_path:="$MAP" include_rviz:=true

tb3_house_good:  
MAP="$(ros2 pkg prefix lab5_nav)/share/lab5_nav/maps/tb3_house_good.yaml"  
ros2 launch lab5_nav nav_bringup.launch.py map_path:="$MAP" include_rviz:=true

tb3_house_bad:  
MAP="$(ros2 pkg prefix lab5_nav)/share/lab5_nav/maps/tb3_house_bad.yaml"  
ros2 launch lab5_nav nav_bringup.launch.py map_path:="$MAP" include_rviz:=true

Lab Report:   
Uploaded as a PDF to the Lab 5 repository.

