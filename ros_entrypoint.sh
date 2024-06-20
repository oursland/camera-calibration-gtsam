#!/bin/bash
set -e

# setup ros2 environment
source "/workspace/install/setup.bash"

ros2 launch mocap_calibration intrinsic_calibration.launch.py &
ros2 bag play /rosbags/rosbag2_2024_06_20-01_19_11
ros2 service call /feynman/calibrate mocap_msgs/srv/Calibrate
