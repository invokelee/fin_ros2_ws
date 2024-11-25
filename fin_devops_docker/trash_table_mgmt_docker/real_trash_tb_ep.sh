#! /bin/bash
echo "[$(date +'%F %T')] In Real Robot: Starting Trash Table detection and moving app for Cleaner robot..."
source /ros2_ws/install/setup.bash && ros2 launch trash_table_detection real_trash_table_app.launch.xml
# ros2 run trash_table_detection approach_the_table.py --ros-args -p target:=real -p robot:=cleaner_2
# ros2 run trash_table_detection trash_table_mover_as.py --ros-args -p target:=real -p robot:=cleaner_2