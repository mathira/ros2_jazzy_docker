# ROS2 Jazzy Dev Container

A Docker dev environment for **ROS 2 Jazzy** with **Gazebo Harmonic**, **colcon**, and **browser-based VNC** access.

## Folder structure

```
ros2_jazzy_docker/
├── .devcontainer/
│   ├── devcontainer.json       # Dev Container config (VS Code / opencode)
│   ├── Dockerfile              # Image: ROS 2 Jazzy + Gazebo + VNC
│   ├── entrypoint.sh           # Sources ROS and starts VNC on boot
│   └── scripts/
│       ├── vnc-start.sh        # Starts Xvfb + x11vnc + noVNC
│       ├── post-create.sh      # One-time setup (VNC password, rosdep, .bashrc)
│       └── build.sh            # colcon build wrapper
└── ros2_ws/
    └── src/                    # <-- YOUR ROS packages (shared with container at /ros2_ws/src)
```

## What's inside

- Ubuntu Noble + ROS 2 **Jazzy** (`ros:jazzy-ros-base`)
- **colcon**, rosdep, vcstool, CMake, pip
- **Gazebo Harmonic** (`gz-harmonic`) + `ros_gz` bridge (`ros-jazzy-ros-gz`)
- **ros2_control** + controllers, xacro, robot_state_publisher, joint_state_publisher
- teleop_twist_keyboard, rviz2, rqt
- **VNC**: `Xvfb` virtual display + `x11vnc` + **noVNC** (browser access)
- Lightweight Xfce desktop

## Requirements

- Docker Desktop (or Docker Engine) on your Mac/Linux
- VS Code with the **Dev Containers** extension (or `docker run`)

---

## Option A — VS Code Dev Containers (recommended)

1. Open this folder in VS Code.
2. When prompted, click **Reopen in Container** (or `Cmd+Shift+P` → *Dev Containers: Reopen in Container*).
3. The first build downloads ~4 GB, so be patient.
4. Open a terminal in VS Code and verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
gz sim --version
```

## Option B — Docker CLI

```bash
# 1. Build the image (only needed the first time / after changes)
docker build -t ros2-jazzy-dev .devcontainer

# 2. Run the container (mounts ./ros2_ws/src -> /ros2_ws/src)
docker run -dit --name ros2-jazzy \
  -v "$PWD/ros2_ws/src:/ros2_ws/src" \
  -v ros2_jazzy_build:/ros2_ws/build \
  -v ros2_jazzy_install:/ros2_ws/install \
  -v ros2_jazzy_log:/ros2_ws/log \
  -p 6080:6080 -p 5901:5901 \
  --privileged --shm-size=1g \
  ros2-jazzy-dev

# 3. Open a shell inside the container
docker exec -it ros2-jazzy bash
```

> Note: `build`, `install`, and `log` are stored in named Docker volumes so they survive container recreation. Only `src/` is bound to your host folder.

---

## Access the GUI from the browser (VNC)

The container starts `Xvfb` (virtual display) + `x11vnc` + `noVNC` automatically.

1. Open your browser: **http://localhost:6080/vnc.html**
2. Password: **`ros`**
3. You get a full Xfce desktop. Launch Gazebo / rviz2:

```bash
source /opt/ros/jazzy/setup.bash
export DISPLAY=:1

# Launch Gazebo Harmonic
gz sim -s -r shapes.sdf

# Or launch rviz2
rviz2
```

`DISPLAY=:1` is already exported in `.bashrc`.

> The container only has a **virtual** display (Xvfb) — it is not connected to your Mac's screen. VNC/noVNC is the supported way to see the GUI. Direct X11 forwarding to XQuartz on Apple Silicon is possible but fragile and **not** recommended.

## X11 on Mac (M4) — can you use it?

Yes, but with caveats:

- Install [XQuartz](https://www.xquartz.org/) and run `xhost +localhost` after launching it.
- Docker Desktop on Apple Silicon runs containers in a VM, so `DISPLAY=:0` (native macOS sockets) **won't work**. You must pass:
  ```bash
  -e DISPLAY=host.docker.internal:0
  ```
- macOS Ventura+ blocks X11 socket access; even when it works, rendering is software-only (slow for Gazebo).

**Recommendation:** use the built-in VNC/noVNC (Option A/B above). It works reliably on M4 and is already configured.

## Building your workspace

Put your ROS 2 packages in `ros2_ws/src/` on the host. Then build:

```bash
# Inside the container
bash /home/ros/.devcontainer/scripts/build.sh
# equivalent to:
#   source /opt/ros/jazzy/setup.bash
#   cd /ros2_ws && colcon build --symlink-install
```

After building, source the workspace:

```bash
source /ros2_ws/install/setup.bash
```

## Manual package example

```bash
cd /ros2_ws/src
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run my_package <node>
```

## Python turtlesim example (`turtle_py`)

Python package with two nodes: a **keyboard teleop** for the turtle and a **spawn** service client.

```bash
# 1. Build (from inside the container)
source /opt/ros/jazzy/setup.bash
cd /ros2_ws && colcon build --symlink-install --packages-select turtle_py
source /ros2_ws/install/setup.bash

# 2. In terminal 1: start the simulator (visible in the VNC browser at http://localhost:6080/vnc.html)
export DISPLAY=:1
ros2 run turtlesim turtlesim_node

# 3. In terminal 2: spawn a second turtle (optional)
export DISPLAY=:1
ros2 run turtle_py spawn_turtle

# 4. In terminal 3: teleoperate the turtle with WASD / arrow keys (q to stop)
ros2 run turtle_py teleop_turtle
```

> `teleop_turtle` publishes `Twist` to `/turtle1/cmd_vel`. Run it in an interactive terminal (`docker exec -it ros2-jazzy bash`) so it can read the keyboard.

## Notes

- Default VNC password: `ros` — change it inside the container with:
  ```bash
  vncpasswd /home/ros/.vnc/passwd
  ```
- To rebuild after changing the Dockerfile:
  ```bash
  docker build -t ros2-jazzy-dev .devcontainer
  docker rm -f ros2-jazzy && <re-run the docker run command above>
  ```
- `--privileged` is used so Gazebo can access GPU/device nodes; drop it if you don't need it.
