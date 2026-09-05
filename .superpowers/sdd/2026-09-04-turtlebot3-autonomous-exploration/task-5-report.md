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
