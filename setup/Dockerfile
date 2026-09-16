# Reproducible ROS 2 Jazzy development image.
# This is intentionally a regular Docker image; it does not depend on VS Code.
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl vim nano htop ca-certificates gnupg \
    python3-pip python3-venv python3-argcomplete python3-pytest \
    python3-vcstool python3-rospkg python3-colcon-common-extensions \
    python3-rosdep python3-numpy python3-yaml python3-opencv \
    libgl1 libfltk1.3-dev libxcomposite1 libxdamage1 \
    xfwm4 xfce4-panel xfce4-terminal dbus-x11 xvfb x11vnc novnc websockify \
    sudo bash-completion less \
    && rm -rf /var/lib/apt/lists/*

ARG TORCH_VERSION=2.10.0
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --index-url https://download.pytorch.org/whl/cpu "torch==${TORCH_VERSION}"

RUN wget -q https://packages.osrfoundation.org/gazebo.gpg \
      -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic ros-jazzy-ros-gz ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-teleop-twist-keyboard ros-jazzy-turtlesim ros-jazzy-rviz2 \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ackermann-msgs \
      python3-pil \
    && rm -rf /var/lib/apt/lists/*

RUN userdel ubuntu 2>/dev/null || true \
    && groupdel ubuntu 2>/dev/null || true \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN su - ${USERNAME} -c 'export CODEX_NON_INTERACTIVE=1; curl -fsSL https://chatgpt.com/codex/install.sh | sh'
ENV PATH=/home/${USERNAME}/.local/bin:${PATH}
ENV PATH=/home/${USERNAME}/setup:${PATH}

COPY setup/container-setup.sh /home/${USERNAME}/setup/container-setup.sh
COPY setup/build.sh /home/${USERNAME}/setup/build.sh
COPY setup/vnc-start.sh /home/${USERNAME}/setup/vnc-start.sh
COPY setup/web-terminal.sh /home/${USERNAME}/setup/web-terminal.sh
RUN chmod +x /home/${USERNAME}/setup/*.sh \
    && mkdir -p /ros2_ws/src /ros2_ws/build /ros2_ws/install /ros2_ws/log /home/${USERNAME}/.codex \
    && chown -R ${USERNAME}:${USERNAME} /ros2_ws /home/${USERNAME}/setup /home/${USERNAME}/.codex \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'source /ros2_ws/install/setup.bash 2>/dev/null || true' >> /home/${USERNAME}/.bashrc \
    && echo 'export DISPLAY=:1' >> /home/${USERNAME}/.bashrc

COPY setup/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR /ros2_ws
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]

LABEL org.opencontainers.image.description="ROS 2 Jazzy, Gazebo Harmonic and project dependencies"
