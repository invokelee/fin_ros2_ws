#! /bin/bash
echo "[$(date +'%F %T')] In Simulation: Starting web tools and web server for Cleaner robot..."
source /ros2_ws/install/setup.bash && source /simulation_ws/install/setup.bash && ros2 launch trash_table_detection sim_web_app.launch.xml &
sleep 5
cd /webpage_ws/cleanerbot_webapp && python3 -m http.server 7000