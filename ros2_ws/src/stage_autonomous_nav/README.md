# Stage autonomous navigation

This package launches the Stage cave simulation together with the Nav2 map,
planner, controller, RViz, and the simulation-only ground-truth localizer.

## Demo

<video controls width="720">
  <source src="assets/stage-ros2.mp4" type="video/mp4">
  Tu visor no permite reproducir el video aquí. Abre [stage-ros2.mp4](assets/stage-ros2.mp4).
</video>

## Prerequisites

Run these commands inside the ROS 2 Jazzy dev container. The container's
existing VNC/noVNC setup provides the GUI display (`DISPLAY=:1`) at
<http://localhost:6080/vnc.html> (password `ros`).

Import the Stage and `stage_ros2` sources listed in `stage_nav.repos`, install
their dependencies, then build and source the workspace:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
vcs import src < stage_nav.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run `vcs import` only if `src/Stage` and `src/stage_ros2` do not already
exist. To update an existing checkout, use `git pull` in each repository.

If Stage reports `Could NOT find FLTK`, install its development dependency,
then rebuild:

```bash
sudo apt-get update
sudo apt-get install -y libfltk1.3-dev
colcon build --symlink-install
source install/setup.bash
```

## Run the integrated navigation demo

Start the complete simulation with:

```bash
export DISPLAY=:1
ros2 launch stage_autonomous_nav cave_navigation.launch.py
```

Stage and RViz are visible in the existing VNC/noVNC desktop. In RViz,
select **2D Goal Pose**, click-drag a free location in the cave to set a goal,
and inspect `/plan` and `/base_scan`. Wait for Nav2 to report the goal result.

The `cave` world uses a single TF tree whose frames are prefixed with
`robot_0/`: `robot_0/odom`, `robot_0/base_link`, and `robot_0/laser`.
The launch configures Nav2 with those names automatically. If logs still say
`Invalid frame ID "odom"` or `Invalid frame ID "base_link"`, stop the launch,
rebuild `stage_autonomous_nav`, source `install/setup.bash` again, and relaunch:

```bash
colcon build --symlink-install --packages-select stage_autonomous_nav
source install/setup.bash
ros2 launch stage_autonomous_nav cave_navigation.launch.py
```

The `/ground_truth` localization input is **simulation-only**. For real
hardware, replace it with AMCL or another localization source that provides
the robot pose and the required `map -> odom` transform.

## Acceptance checks

With a running launch, issue a free-space goal such as `(-1.0, -5.5)` with a
valid orientation. Confirm the following from another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic echo /tf --once
ros2 topic echo /base_scan --once
ros2 topic echo /cmd_vel --once
ros2 lifecycle get /map_server
ros2 lifecycle get /controller_server
```

`map -> robot_0/odom` should exist in TF, `/base_scan` should contain ranges,
Nav2 should emit nonzero velocity only after a goal is sent, and both lifecycle
nodes should be `active`. Send a second goal whose route passes near `(5, 4)`
and visually confirm in Stage that the robot avoids the block.

## Tests

The package tests can be run with:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select stage_autonomous_nav
source install/setup.bash
colcon test --packages-select stage_autonomous_nav
colcon test-result --verbose
```
