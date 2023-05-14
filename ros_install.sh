#!/bin/bash

# Define colors for output messages
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Prompt user to choose version
echo -e "${YELLOW}Which version of the package would you like to install?"
echo -e "1. v1.0.0"
echo -e "2. v2.0.0${NC}"
read -n 1 -p "Enter your choice [1 or 2]: " choice

# Set branch variable based on user's choice
if [[ $choice -eq 1 ]]; then
    branch="main"
    elif [[ $choice -eq 2 ]]; then
    branch="v2.0.0"
else
    echo -e "${RED}Invalid choice. Exiting.${NC}"
    exit 1
fi

# Check if workspace exists
if [[ ! -d ~/ros_ws ]]; then
    echo -e "${YELLOW}Workspace does not exist. Creating new workspace...${NC}"
    mkdir -p ~/ros_ws/src
else
    echo -e "${GREEN}Workspace already exists.${NC}"
fi

# Check if package exists
if [[ ! -d ~/ros_ws/src/Experimental-Robotics-Project.2022 ]]; then
    echo -e "${YELLOW}Package does not exist. Cloning package...${NC}"
    cd ~/ros_ws/src
    git clone https://github.com/Omotoye/Experimental-Robotics-Project.2022.git
else
    echo -e "${GREEN}Package already exists.${NC}"
fi

# Checkout desired branch
cd ~/ros_ws/src/Experimental-Robotics-Project.2022
git checkout $branch

# Build the workspace
source /opt/ros/noetic/setup.bash
cd ~/ros_ws/
catkin_make

# Print final message to user
if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}ROS package successfully installed!${NC}"
else
    echo -e "${RED}Error installing ROS package.${NC}"
fi
