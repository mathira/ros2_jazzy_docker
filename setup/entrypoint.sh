#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
bash /home/ros/setup/vnc-start.sh || true
exec "$@"
