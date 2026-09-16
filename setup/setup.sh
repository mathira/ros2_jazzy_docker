#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
docker compose -f docker-compose.yml build
docker compose -f docker-compose.yml run --rm ros2 /home/ros/setup/container-setup.sh
docker compose -f docker-compose.yml run --rm ros2 /home/ros/setup/build.sh
docker compose -f docker-compose.yml up -d
echo 'Installation complete. Enter with: docker compose -f setup/docker-compose.yml exec ros2 bash'
