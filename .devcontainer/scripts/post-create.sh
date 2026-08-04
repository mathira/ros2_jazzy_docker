#!/usr/bin/env bash
# One-time setup after the container is created.
set -e

echo "Setting up VNC password..."
mkdir -p /home/ros/.vnc
if [ ! -f /home/ros/.vnc/passwd ]; then
    # Default password: ros. Change with: vncpasswd /home/ros/.vnc/passwd
    echo "ros" | x11vnc -storepasswd "ros" /home/ros/.vnc/passwd
fi
chmod 600 /home/ros/.vnc/passwd

echo "Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

echo "Sourcing ROS environment..."
echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc
echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/ros/.bashrc
echo "export DISPLAY=:1" >> /home/ros/.bashrc

echo "Creating default colcon profile..."
mkdir -p /ros2_ws/src

echo "Post-create setup complete."
