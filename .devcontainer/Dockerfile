# ROS2 Jazzy development container with Gazebo and VNC support
ARG BASE_IMAGE=ros:jazzy-ros-base

FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Install system dependencies, ROS 2 dev tools, and desktop/GUI packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # Build & dev tools
    build-essential cmake git wget curl vim nano htop \
    python3-pip python3-venv python3-argcomplete python3-pytest \
    python3-vcstool python3-rospkg python3-colcon-common-extensions \
    python3-rosdep \
    # GUI / simulation libs
    libgl1 libxcomposite1 libxdamage1 \
    # Lightweight desktop for VNC
    xfwm4 xfce4-panel xfce4-terminal dbus-x11 \
    # VNC server (x11vnc), virtual X display (Xvfb) and noVNC web client
    xvfb x11vnc novnc websockify \
    # misc
    sudo bash-completion less \
    && rm -rf /var/lib/apt/lists/*

# Add OSRF (Gazebo) apt repository - required for Gazebo Harmonic on Noble
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" > /etc/apt/sources.list.d/gazebo-stable.list

# Install Gazebo Harmonic + ROS 2 integration (ros_gz), ros2_control and nav/teleop deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-turtlesim \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-ackermann-msgs \
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

# Create user 'ros' with uid/gid 1000 (base image ships an 'ubuntu' user on 1000, replace it)
RUN userdel ubuntu 2>/dev/null || true \
    && groupdel ubuntu 2>/dev/null || true \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Copy helper scripts
COPY scripts /home/${USERNAME}/.devcontainer/scripts
RUN chmod +x /home/${USERNAME}/.devcontainer/scripts/*.sh

# Create workspace
RUN mkdir -p /ros2_ws/src

# Entrypoint: source ROS env and start services, then exec the real CMD
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR /ros2_ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]

LABEL org.opencontainers.image.description="ROS 2 Jazzy with Gazebo and browser VNC (noVNC) access"
