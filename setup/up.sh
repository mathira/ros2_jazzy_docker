#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
docker compose -f docker-compose.yml up -d --build
echo 'Container running: docker compose -f setup/docker-compose.yml exec ros2 bash'
echo 'GUI: http://localhost:6080/vnc.html (password: ros)'
