#!/usr/bin/env bash
# Install the required turtlesim package (debian) so the sim and services exist.
source /opt/ros/jazzy/setup.bash
# turtlesim is a separate package in Jazzy; ensure it is installed.
if ! ros2 pkg prefix turtlesim > /dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-turtlesim
fi