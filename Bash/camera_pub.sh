#! /bin/bash

set -e  # Exit on any command failure


cd ~/ros2_ws/ || { echo "Failed to change directory"; exit 1; }
source install/setup.bash || { echo "Failed to source setup.bash"; exit 1; }
ros2 run talker_rgb || { echo "Failed to start Subscriber"; exit 1; }
