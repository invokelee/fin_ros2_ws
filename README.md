# Cafeteria Cleanerbot Table Picker package
## Demonstration 
<br/>

### Pre-requistion:
- Required Ubuntu 22.04 installed system
<br />

### Run Demo Steps
<br/>

### 1. Clone the source
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/invokelee/fin_ros2_ws.git .
```
### 2. Install docker, if not installed on your PC
```bash
cd ~/ros2_ws/src/fin_devops_docker/
./start_docker_v2.sh
newgrp docker
```

### 3. Run gzclient or install X11 client 
```bash
export DISPLAY=<local pc ip address>:0
gzclient
```

### 4. Launch the service (Run docker compose up)
```bash
cd ~/ros2_ws/src/fin_devops_docker/
docker compose -f sim_final_docker_compose.yml up
```
** Wait for the simulator and localization server (rviz with map) to start.
### 5. Run a browser(ex, chrome) with web server address 
```bash
web address : http://<linux ip>:7000/
ROS bridge address : ws://<linux ip>:9090
```
<p align="center">
  <img src="https://raw.githubusercontent.com/invokelee/fin_ros2_ws/refs/heads/main/doc/images/99-02-browser_and_bridge.png" width="50%"/>
 <br/>
  Fig. Input Web page address and ROSBridge address 
</p>

### 6. Check the gazebo simulator and  a browser(ex, chrome) with web server address 
- Check if the Gazebo simulator and RViZ (map) are displayed
- Connect to the Web Server with a browser such as Chrome
- Set the ROS Bridge address to connect to the service
- Check various functions after to connecting

```bash
web address : http://<linux ip>:7000/
ROS bridge address : ws://<linux ip>:9090
```
<p align="center">
  <img src="https://raw.githubusercontent.com/invokelee/fin_ros2_ws/refs/heads/main/doc/images/99-01-simulation-demo.png" width="100%"/>
 <br/>
  Fig. Gazebo simulator and RViZ and Web Application 
</p>

### 7. Shutdown the service (Run docker compose down)
```bash
cd ~/ros2_ws/src/fin_devops_docker/
docker compose -f sim_final_docker_compose.yml down
```
