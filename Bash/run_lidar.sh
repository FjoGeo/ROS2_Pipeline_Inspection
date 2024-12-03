#! /bin/bash

set -e  # Exit on any command failure

sudo chmod 777 /dev/ttyUSB* || { echo "Failed to set permissions"; exit 1; }
cd ~/ros2_ws/ || { echo "Failed to change directory"; exit 1; }
source install/setup.bash || { echo "Failed to source setup.bash"; exit 1; }
ros2 run rp_test talker || { echo "Failed to start ROS node"; exit 1; }
