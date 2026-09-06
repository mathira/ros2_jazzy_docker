# Gazebo training reset verification

Tested in the existing Jazzy ARM64 container on 2026-09-05 (Montevideo), with
Gazebo Sim 8.11.0, the official TurtleBot3 stage4 world and Burger model from
the `/tmp/tb3-task6-deps` overlay. Diagnostic worlds used distinct Gazebo
partitions. No Stage package or upstream TurtleBot3 files were modified.

## Cause and minimal comparison

A direct Gazebo transport probe subscribed to `/odom`, `/scan`, `/clock`, and
`/world/dqn/pose/info`. It loaded stage4, created Burger through `/world/dqn/create`
as the official launch does, moved it forward at 0.1 m/s for three seconds,
stopped it, and invoked each operation below in a fresh simulation.

All control requests returned `data: true`. The counts below are messages
received during the five-second wall-clock observation after the request.
The robot had accumulated about 0.296 m of odometry before each operation.

| Operation | Odometry | Scans | Result |
| --- | ---: | ---: | --- |
| `reset.all` after dynamic creation | 0 | 24 | Clock rewinds; Burger vanishes from pose publications. |
| `reset.time_only` after dynamic creation | 0 | 24 | Same deletion and clock rewind as full reset. |
| `reset.model_only` | 245 | 24 | Robot position, accumulated odometry, clock, and obstacles continue. |
| `set_pose` to the origin | 250 | 25 | Physical pose returns near the origin, but odometry stays near 0.297 m and obstacles continue. |
| `reset.all`, Burger included in initial world | 244 | 24 | Robot remains present; odometry returns to approximately zero. |
| Second `reset.all`, same initial-world robot | 244 | 24 | Same successful robot and odometry restoration. |

Thus the stopped odometry was caused by resetting away a dynamically created
robot, rather than an inherent inability of DiffDrive to operate after every
full reset. Gazebo's
[SimulationRunner](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/SimulationRunner.cc)
restores the initial entity-component state on rewind. The same implementation
maps `all` and `time_only` to rewind and explicitly warns that model-only reset
is unsupported. A service acknowledgement alone does not prove the requested
episode state exists.

## Implemented behavior

Only training uses `TrainingStage4LaunchSource`. It copies the official stage4
world into a temporary directory and adds the official Burger model at
`0 0 0.01 0 0 0`. It keeps upstream robot state publication and bridge actions,
omits the later `/create` process, and cleans the temporary directory when
launch shuts down. The mission launcher retains its existing behavior.

The trainer still requests `ControlWorld.world_control.reset.all = true` for
every episode. Controller disable acknowledgement, mapper and observation
reset epochs, DDS publication cutoffs, and post-reset sensor gates remain
unchanged.

## ROS integration result

After rebuilding with `colcon build --packages-select turtleboot3_autonomous_nav
--symlink-install`, the run used:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/tb3-task6-deps/install/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=86 GZ_PARTITION=training_reset_workaround2 TURTLEBOT3_MODEL=burger
ros2 launch turtleboot3_autonomous_nav training.launch.py \
  training_config:=/ros2_ws/src/turtleboot3_autonomous_nav/config/smoke_training.yaml \
  model_directory:=/tmp/gazebo-reset-workaround-models \
  episodes:=2 use_gui:=false
```

All four episodes logged startup after reset acknowledgements and fresh odometry
and scan, then finished at 20 steps. The trainer reported its configured limit
and exited cleanly. Training/evaluation coverage fractions were approximately
0.054, 0.042, 0.052, and 0.043. The final saved evaluation metadata contained
`episode: 4`, `global_steps: 80`, and `mean_coverage: 0.042518749833106995`.
`best.pt` and `best.metrics.json` were created and `load_checkpoint` successfully
reloaded the checkpoint.

The package suite passed: **69 tests**. The new launch regression test first
failed without the initial-world source, then passed. It verifies that removing
the added Burger include leaves the original world's parsed XML unchanged,
that Burger starts at the expected pose, and that its upstream bridge remains
while dynamic creation is omitted.

The smoke model is an integration artifact, not evidence of useful learned
exploration performance. No per-episode simulator relaunch was needed.
