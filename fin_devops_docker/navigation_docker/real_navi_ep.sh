#! /bin/bash
echo "[$(date +'%F %T')] In Real Robot: Starting Navigation(localization & path_planner) for Cleaner robot..."
source /ros2_ws/install/setup.bash && ros2 launch trash_table_detection real_path_planner_w_loc.launch.xml
