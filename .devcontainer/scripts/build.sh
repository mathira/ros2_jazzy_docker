#!/usr/bin/env bash
# Build the ROS 2 workspace with colcon.
set -e

source /opt/ros/jazzy/setup.bash

cd /ros2_ws
mkdir -p /ros2_ws/src

echo "Building workspace with colcon..."
colcon build --symlink-install "$@"

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

echo "Build complete. Source with: source /ros2_ws/install/setup.bash"