#! /bin/bash

set -e  # Exit on any command failure

sudo chmod 777 /dev/ttyUSB* || { echo "Failed to set permissions"; exit 1; }
cd ~/ros2_ws/ || { echo "Failed to change directory"; exit 1; }
source install/setup.bash || { echo "Failed to source setup.bash"; exit 1; }
cd ~/ros2_ws/launch/ || { echo "Failed to change directory 2"; exit 1; }
ros2 launch 2lidar_RGB_ACC_GYRO.py || { echo "Failed to start launch file"; exit 1; } 