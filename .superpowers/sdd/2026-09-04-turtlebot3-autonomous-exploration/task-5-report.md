# Task 5 — Gazebo episode DQN trainer

## Delivered

- Added `dqn_trainer.py`, with dependency-free reward, terminal-condition,
  checkpoint, replay TD-update, and Gazebo-reset helpers plus the ROS 2
  `dqn_trainer` adapter.
- The adapter consumes `/dqn_observation`, `/coverage_metrics`,
  `/safety_intervention`, and `/recovery_active`; publishes exploration
  actions and numeric `/training_metrics`; records replay transitions; trains
  with a target network; and evaluates with epsilon zero.
- Evaluation checkpoints are written only for a strictly improved mean
  coverage score: `models/best.pt` and `models/best.metrics.json`.
- World reset uses `ros_gz_interfaces/srv/ControlWorld` at
  `/world/dqn/control`, setting the exact Gazebo flag
  `request.world_control.reset.all = True`, then waits for fresh `/odom` and
  `/scan` before an episode begins.
- Added all trainer parameters and reward weights to `config/training.yaml`.
- Added behavioral tests for reward shaping, terminal conditions, reset-all,
  checkpoint improvement gating, and replay optimization.

## Verification

```text
docker run ... ros2-jazzy-dev:codex-check ... \
  colcon build --packages-select turtleboot3_autonomous_nav
```

Succeeded: `turtleboot3_autonomous_nav` built.

```text
pytest src/turtleboot3_autonomous_nav/test -v
```

Succeeded: 28 passed.

```text
ros2 interface show ros_gz_interfaces/srv/ControlWorld
```

Confirmed that `WorldControl.reset` has the `bool all` field used by the
trainer.

`black --check` and `git diff --check` also succeeded.

## Integration boundary

No Gazebo or launch file was started or modified.  Task 6 owns launch assets,
and this task was explicitly constrained not to start them; therefore a
headless episode checkpoint was not executed in this task.  The trainer creates
the model files on the first improved zero-epsilon evaluation when launched by
Task 6.

## Review follow-up: episode isolation

- Added `/coverage_mapper/reset` as a `std_srvs/srv/Empty` service.  Its handler
  clears occupancy values, free/occupied evidence, known-cell count, and the
  cached odometry pose, then publishes a zero coverage metric.
- The trainer now implements the ordered reset transaction:
  `ControlWorld` success ACK → mapper reset success ACK → post-ACK odometry and
  scan callbacks.  It snapshots callback sequence numbers after the mapper ACK,
  so callbacks already observed before that point cannot satisfy the fresh-data
  gate.  Local coverage and its reward baseline are reset to `0.0` immediately
  before the episode is allowed to start.
- Added regression tests for the acknowledgement/sequence gate and for
  clearing grid coverage and accumulated cell evidence.

Verification after the follow-up: the Jazzy container built
`turtleboot3_autonomous_nav`; all package tests passed (`30 passed`), and
`black --check` plus `git diff --check` succeeded.
