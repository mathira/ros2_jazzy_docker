#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
cd /ros2_ws
mkdir -p src build install log

if [ -f stage_nav.repos ]; then
  vcs import --skip-existing src < stage_nav.repos
fi
if [ -f src/turtleboot3_autonomous_nav/dependencies/turtlebot3_jazzy.repos ]; then
  for repository in turtlebot3 turtlebot3_msgs turtlebot3_simulations; do
    if [ -d "src/$repository" ] && [ -z "$(find "src/$repository" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
      rmdir "src/$repository"
    fi
  done
  vcs import --skip-existing src < src/turtleboot3_autonomous_nav/dependencies/turtlebot3_jazzy.repos
fi

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
  --skip-keys 'ament_python ament_pytest ament_cmake_clang_format ament_cmake_pycodestyle turtlebot3_gazebo tf_transformations hls_lfcd_lds_driver cartographer_ros dynamixel_sdk libtool libtool-bin'
echo 'Container setup complete. Run: setup/build.sh'
