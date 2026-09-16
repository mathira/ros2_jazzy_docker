# turtle_py — Python Turtlesim Teleop & Spawn

A small ROS 2 (Jazzy) Python package with two nodes to play with **turtlesim**:

- **`teleop_turtle`** — keyboard teleoperation of a turtle (publishes `Twist` to `/turtle1/cmd_vel`)
- **`spawn_turtle`** — spawns a second turtle using the `/spawn` service

## Prerequisites

- The project container from the root `README` is running (`ros2-jazzy-project`).
- `ros-jazzy-turtlesim` is installed inside the container.

## 1. Build the package

The web console is available inside the project container. Open
<http://localhost:6080/vnc.html> (password `ros`) and use the automatically
opened **ROS 2 Jazzy - project terminal**. It starts in `/ros2_ws` with ROS 2
loaded. A host terminal remains available as an alternative:

```bash
docker compose -f setup/docker-compose.yml exec ros2 bash
```

Then build and source your workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-select turtle_py
source /ros2_ws/install/setup.bash
```

> Build only this package to keep it fast. To build everything: `colcon build --symlink-install`.

## 2. Launch the simulator

Start turtlesim. Because this is a headless container, the window appears in your **browser** via VNC at:

```
http://localhost:6080/vnc.html     (password: ros)
```

```bash
export DISPLAY=:1
ros2 run turtlesim turtlesim_node
```

You should see a turtle at the center of a blue window.

## 3. Teleoperate the turtle

In the Xfce terminal inside noVNC (or another interactive container terminal):

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run turtle_py teleop_turtle
```

Control keys:

| Key            | Action         |
|----------------|----------------|
| `w` / UP arrow | move forward   |
| `s` / DOWN     | move backward  |
| `a` / LEFT     | turn left      |
| `d` / RIGHT    | turn right     |
| `q` / SPACE    | stop           |
| `Ctrl-C`       | quit           |

While running it **publishes** `Twist` messages continuously on `/turtle1/cmd_vel`.

> ⚠️ Keyboard reading needs a real TTY. The terminal opened inside noVNC is a
> real interactive terminal, so focus that window before pressing movement
> keys.

## 4. (Optional) Spawn a second turtle

In a third terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run turtle_py spawn_turtle
```

This calls the `/spawn` service and creates **`turtle2`** at `(5, 5)`. If a turtle with that name already exists the service returns an empty name (logged as a warning).

To drive `turtle2` instead, run the teleop with the topic remapped:

```bash
ros2 run turtle_py teleop_turtle --ros-args -r /turtle1/cmd_vel:=/turtle2/cmd_vel
```

## Inspecting topics while it runs

```bash
ros2 topic list                        # show all topics
ros2 topic info /turtle1/cmd_vel       # publisher/subscriber details
ros2 topic echo /turtle1/cmd_vel       # live Twist messages
ros2 node info /teleop_turtle          # info about the teleop node
```

## Package layout

```
src/turtle_py/
├── package.xml            # manifest (rclpy, geometry_msgs, std_msgs, turtlesim)
├── setup.py               # console scripts -> entry points
├── resource/turtle_py
├── setup.cfg
└── turtle_py/
    ├── __init__.py
    ├── teleop_turtle.py   # keyboard teleop -> /turtle1/cmd_vel
    └── spawn_turtle.py    # /spawn service client
```

> Note: The nodes are installed under `lib/turtle_py` via `setup.py` entry points, so they run with `ros2 run turtle_py <node>` after sourcing `install/setup.bash`.
