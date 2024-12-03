#! /bin/bash

set -e  # Exit on any command failure


cd ~/ros2_ws/ || { echo "Failed to change directory"; exit 1; }
source install/setup.bash || { echo "Failed to source setup.bash"; exit 1; }
cd ~/ros2_ws/launch/ || { echo "Failed to change directory 2"; exit 1; }
ros2 launch launch_dual_camera.py
