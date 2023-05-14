#!/bin/bash

# Create a folder for your catkin workspace
mkdir -p ~/ros_ws/src

# Clone the package repository
cd ~/ros_ws/src
git clone https://github.com/Omotoye/Experimental-Robotics-Project.2022.git
# git checkout v1.0.0

# Build the workspace
source /opt/ros/noetic/setup.bash
cd ~/ros_ws/
catkin_make
