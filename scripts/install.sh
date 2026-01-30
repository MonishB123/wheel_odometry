#!/bin/bash
set -e

sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox

cd ~/ros2_ws
colcon build --packages-select wheel_odometry
source install/setup.bash
