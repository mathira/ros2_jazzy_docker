#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
if [ "$#" -eq 0 ]; then
  colcon build --symlink-install --packages-up-to \
    stage_autonomous_nav autonomous_navigation_pria turtleboot3_autonomous_nav turtle_py
else
  colcon build --symlink-install "$@"
fi
source install/setup.bash
echo 'Workspace built and sourced.'
