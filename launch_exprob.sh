#!/bin/bash

# Define colors for output messages
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Find available terminal emulator
if command -v gnome-terminal &> /dev/null
then
    terminal=gnome-terminal
elif command -v terminator &> /dev/null
then
    terminal=terminator
elif command -v konsole &> /dev/null
then
    terminal=konsole
elif command -v xterm &> /dev/null
then
    terminal=xterm
else
    echo -e "${RED}No terminal emulator found.${NC}"
    exit 1
fi

# Source the setup.bash file in both terminals
source_cmd="source ~/catkin_ws/devel/setup.bash"
cmd1="$terminal --title='Knowledge Package' --working-directory='~/catkin_ws' -e 'bash -c \"$source_cmd; roslaunch exprob_knowledge knowledge.launch; exec bash\"'"
cmd2="$terminal --title='Surveillance Package' --working-directory='~/catkin_ws' -e 'bash -c \"$source_cmd; roslaunch exprob_logic surveillance.launch; exec bash\"'"
cmd3="$terminal --title='SMACH Viewer' --working-directory='~/catkin_ws' -e 'bash -c \"$source_cmd; rosrun smach_viewer smach_viewer.py; exec bash\"'"

# Launch the terminals

# Launch knowledge package
echo -e "${YELLOW}Launching knowledge package...${NC}"
eval "$cmd1" &
sleep 5

# Launch surveillance package
echo -e "${YELLOW}Launching surveillance package...${NC}"
eval "$cmd2" &
sleep 2

# Optional: launch smach viewer
echo -e "${YELLOW}Launching smach viewer...${NC}"
eval "$cmd3" &

# Print final message to user
if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}ROS project launched successfully!${NC}"
else
    echo -e "${RED}Error launching ROS project.${NC}"
fi
