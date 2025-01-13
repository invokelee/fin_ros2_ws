sudo pip3 install pyquaternion
sudo apt-get update
sudo apt install ros-humble-irobot-create-msgs
ros2 action send_goal /cleaner_2/dock irobot_create_msgs/action/Dock "{}"
ros2 action send_goal /cleaner_2/undock irobot_create_msgs/action/Undock "{}"
-------------------------------------------------------
ros2 topic echo /cleaner_2/battery_state
python3 ~/ros2_ws/src/trash_table_detection/scripts/scan_info.py --ros-args -p robot_name:=cleaner_2

==================================
Real Robot:
-----------
ros2 launch trash_table_detection real_path_planner_w_loc.launch.xml

ros2 launch trash_table_detection real_trash_table_app.launch.xml
-------------
ros2 run trash_table_detection approach_the_table.py --ros-args -p target:=real -p robot:=cleaner_2
ros2 run trash_table_detection trash_table_mover_as.py --ros-args -p target:=real -p robot:=cleaner_2


ros2 launch trash_table_detection real_web_app.launch.xml

cd ~/webpage_ws/cleanerbot_webapp
python3 -m http.server 7000

webpage_address
rosbridge_address
