# Stage autonomous navigation

This package launches the Stage cave simulation together with the Nav2 map,
planner, controller, RViz, and the simulation-only ground-truth localizer.

## Prerequisites

Run these commands inside the ROS 2 Jazzy dev container. The container's
existing VNC/noVNC setup provides the GUI display (`DISPLAY=:1`) at
<http://localhost:6080/vnc.html> (password `ros`).

Import the Stage and `stage_ros2` sources listed in `stage_nav.repos`, install
their dependencies, then build the workspace:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
vcs import src < stage_nav.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If the sources were imported previously, `vcs import` is safe to rerun; it
updates the checkout to the versions specified by the repos file.

## Run the integrated navigation demo

Start the complete simulation with:

```bash
export DISPLAY=:1
ros2 launch stage_autonomous_nav cave_navigation.launch.py
```

Stage and RViz are visible in the existing VNC/noVNC desktop. In RViz,
select **2D Goal Pose**, click-drag a free location in the cave to set a goal,
and inspect `/plan` and `/base_scan`. Wait for Nav2 to report the goal result.

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

`map -> odom` should exist in TF, `/base_scan` should contain ranges, Nav2
should emit at least one nonzero velocity while travelling, and both lifecycle
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
