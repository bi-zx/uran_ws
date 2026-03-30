#!/bin/bash
set -e
source /opt/ros2/galactic/setup.bash
source /opt/ros2/cyberdog/setup.bash
source /opt/ros2/cyberdog/local_setup.bash
source /SDCARD/uran_ws/install/setup.bash
exec ros2 run uran_media uran_media_node