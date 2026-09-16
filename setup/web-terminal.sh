#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

cd /ros2_ws
printf '\nROS 2 Jazzy workspace ready.\n'
printf 'Use: ros2 run turtle_py teleop_turtle\n\n'
exec bash
