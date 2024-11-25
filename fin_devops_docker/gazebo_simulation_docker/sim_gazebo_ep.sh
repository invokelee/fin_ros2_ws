#! /bin/bash
echo "[$(date +'%F %T')] Starting ROS2 Simulation of Cleaner robot..."
source /simulation_ws/install/setup.bash && ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml
# update test