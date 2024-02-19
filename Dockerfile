# Start from ROS base image
FROM osrf/ros:galactic-desktop

# Make a catkin workspace
WORKDIR /
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Install Git
RUN apt-get update && apt-get install -y git

COPY tortoisebot /ros2_ws/src/tortoisebot

WORKDIR /ros2_ws

# Build your ROS packages
RUN /bin/bash -c "source /opt/ros/galactic/setup.bash; cd /ros2_ws; colcon build"

# Source the workspace every time a new shell is opened in the container
RUN echo source /ros2_ws/install/setup.bash >> ~/.bashrc

RUN apt-get update && apt-get install -y ros-galactic-xacro ros-galactic-urdf

RUN apt-get update && apt-get install -y ros-galactic-joint-state-publisher ros-galactic-robot-state-publisher ros-galactic-gazebo-plugins

# Set the entry point to start a bash shell
CMD /bin/bash -c "source /ros2_ws/install/setup.bash; ros2 launch tortoisebot_waypoints waypoints_set.launch.py use_sim_time:=True"