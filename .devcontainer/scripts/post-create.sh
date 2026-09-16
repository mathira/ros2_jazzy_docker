#!/usr/bin/env bash
# One-time setup after the container is created.
set -e

# Named volumes created by older images can be owned by root.
sudo chown "$(id -u):$(id -g)" /ros2_ws/build /ros2_ws/install /ros2_ws/log

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
for line in \
    'source /opt/ros/jazzy/setup.bash' \
    'source /ros2_ws/install/setup.bash 2>/dev/null || true' \
    'export DISPLAY=:1'; do
    grep -qxF "$line" /home/ros/.bashrc || echo "$line" >> /home/ros/.bashrc
done

echo "Creating default colcon profile..."
mkdir -p /ros2_ws/src

echo "Post-create setup complete."
