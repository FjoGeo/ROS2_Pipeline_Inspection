#! /bin/bash

set -e  # Exit on any command failure

# Launch publisher in the current terminal
gnome-terminal -- bash -c "cd ~/ros2_ws/ && source install/setup.bash && ros2 run my_realsense talker_rgb; exec bash" || { echo "Failed to start Publisher"; exit 1; }

# Launch subscriber in a new terminal
gnome-terminal -- bash -c "cd ~/ros2_ws/ && source install/setup.bash && ros2 run my_realsense sub_rgb; exec bash" || { echo "Failed to start Subscriber"; exit 1; }
