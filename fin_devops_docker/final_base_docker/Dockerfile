FROM osrf/ros:humble-desktop

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

RUN apt update \
# && apt upgrade \
 && apt install -y ros-dev-tools \
#  && apt install -y ros-humble-gazebo-ros-pkgs \
 && apt install -y ros-humble-joint-* \
 && apt install -y ros-humble-cartographer* \
 && apt install -y ros-humble-navigation2 \
 && apt install -y ros-humble-nav2-bringup \
 && apt install -y ros-humble-ros2-control \
 && apt install -y ros-humble-ros2-controllers \
 && apt install -y ros-humble-gazebo-ros2-control \
 && apt install -y ros-humble-xacro \
 && apt install -y ros-humble-topic-tools 

RUN apt install -y python-is-python3 python3-pip vim net-tools
RUN pip install pyquaternion

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN source /opt/ros/humble/setup.bash \
 && mkdir -p /ros2_ws/src \
 && mkdir -p /simulation_ws/src \
 && mkdir -p /webpage_ws/src \
 && cd /ros2_ws/src

WORKDIR /ros2_ws/src
COPY ./ros2_ws/src/cartographer_slam     ./cartographer_slam
COPY ./ros2_ws/src/localization_server   ./localization_server
COPY ./ros2_ws/src/map_server            ./map_server
COPY ./ros2_ws/src/path_planner_server   ./path_planner_server
COPY ./ros2_ws/src/table_find_interface  ./table_find_interface
COPY ./ros2_ws/src/trash_table_detection ./trash_table_detection

RUN cd /simulation_ws/src
WORKDIR /simulation_ws/src
COPY ./simulation_ws/src/async_web_server_cpp   ./async_web_server_cpp
COPY ./simulation_ws/src/course_web_dev_ros     ./course_web_dev_ros
COPY ./simulation_ws/src/rb1_ros2               ./rb1_ros2
COPY ./simulation_ws/src/rosbridge_suite        ./rosbridge_suite
COPY ./simulation_ws/src/tf2_web_republisher_py ./tf2_web_republisher_py
COPY ./simulation_ws/src/universal_robot_ros2   ./universal_robot_ros2
RUN rm -rf ./universal_robot_ros2/gazebo_ros2_control

RUN cd /webpage_ws
WORKDIR /webpage_ws
COPY ./webpage_ws/cleanerbot_webapp   ./cleanerbot_webapp

# Build the Colcon workspace and ensure it's sourced
RUN source /opt/ros/humble/setup.bash \
 && cd /ros2_ws \
 && colcon build

RUN cd /simulation_ws \
 && rosdep fix-permissions && rosdep update \
 && rosdep install --from-paths src -i -y --rosdistro humble --ignore-src

RUN source /opt/ros/humble/setup.bash \
 && cd /simulation_ws \
 && colcon build

# Set the working folder at startup
RUN echo "source /simulation_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws
CMD ["bash"]