#!/bin/bash
set -e
cd /SDCARD/uran_ws
export ROS_DOMAIN_ID=42
source /home/mi/.bashrc
source /opt/ros2/galactic/setup.bash
source /SDCARD/uran_ws/install/setup.bash
ros2 run uran_move uran_move_node