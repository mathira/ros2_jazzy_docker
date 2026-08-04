#!/usr/bin/env bash
# Container entrypoint: source ROS, start VNC, then run the container command.
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Start VNC services in the background
bash /home/ros/.devcontainer/scripts/vnc-start.sh || true

# Execute the real command (default: /bin/bash)
exec "$@"