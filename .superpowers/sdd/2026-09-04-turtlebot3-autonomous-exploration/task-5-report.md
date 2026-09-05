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

## P1 follow-up: delayed ROS publications across reset (2026-09-05)

### Cause and implemented barrier

The callback-sequence gate measured delivery order, not publication order. A
pre-reset `/odom` or `/scan` publication could remain queued until after the
mapper-reset response, populate the cleared grid, and satisfy both trainer
counters. The interrupted follow-up used sensor-header timestamps, but Gazebo
`reset.all` rewinds those simulation timestamps and makes an old high timestamp
appear newer than a current low timestamp.

- `/coverage_mapper/reset` now uses `std_srvs/srv/Trigger`. Its successful
  response carries JSON integer `cutoff_ns` and `epoch` fields. The mapper
  clears the map/evidence and cached pose, publishes zero coverage, and establishes
  the system-time cutoff at the successful response boundary.
- Mapper and trainer subscription callbacks now consume RMW publication
  provenance (`source_timestamp`), which Jazzy rclpy supplies in a dictionary.
  They require publication strictly after the cutoff, including rejection of
  timestamps equal to the boundary and absent/unsupported zero timestamps.
  Callback receipt time is never substituted. The shared helper is pure Python
  and does not add a NumPy or ROS dependency to the trainer helpers.
- The trainer still orders world-reset success, mapper-reset success, then fresh
  odometry and scan. The same publication barrier remains active for observations,
  coverage, and safety/recovery events throughout the episode so delayed old
  publications cannot change training state after the readiness gate opens.
- Corrected the interrupted adapter's undefined `response` variable and retained
  explicit failure/provenance validation before arming the reset gate.
- All public topic names and message types are preserved. The reset service type
  change is coordinated between its only package client and server. No Stage,
  launch, configuration, model, or unrelated workspace files were changed.

This barrier uses ROS publication provenance on the deployment's shared host
system clock; it is independent of `/clock` and ROS `use_sim_time`. ROS defines
`source_timestamp` as the time the publisher published the message
([RMW documentation](https://docs.ros.org/en/rolling/p/rmw/generated/structrmw__message__info__s.html)).
The real DDS regression verifies that clock-domain assumption in the Jazzy
container. The guarantee covers queued ROS publications, not unobservable
buffers inside Gazebo before the bridge publishes a message, nor reconstruction
of already published observation payloads from the builder's upstream caches.
Distributed deployment with unsynchronized machine clocks would need a different
provenance protocol. Unsupported source timestamps safely keep the reset gate
closed.

### Regression and verification evidence

The new tests first failed against the interrupted implementation: both pure
timestamp-boundary assertions failed; the mapper adapter lacked message-info
support; and the trainer's mapper ACK callback raised `NameError`. The real DDS
test also exposed Jazzy's dictionary metadata representation, which was corrected
before final verification.

The regression suite now verifies:

- Delayed old odometry and scans cannot restore map coverage after reset, replace
  a fresh pose, or combine an old pose with a fresh scan; scan-only delivery waits
  for fresh odometry.
- Old publications, zero metadata, and exact-boundary publications cannot start
  an episode. Fresh publications with simulation headers rewound from 90 seconds
  to zero can start it.
- Queued observations, coverage and safety/recovery events cannot contaminate
  the running episode or add replay transitions.
- A real DDS publisher sends old messages before reset; callbacks execute only
  after reset. RMW timestamps remain before the cutoff and the grid stays empty.
  New publications with rewound headers then successfully map cells.
- Malformed reset-provenance responses are rejected, and the gate blocks data
  again when another reset begins.

Final commands and outcomes:

```text
docker run --rm --user root --entrypoint bash \
  -v <repository>/ros2_ws/src/turtleboot3_autonomous_nav:/tmp/ws/src/turtleboot3_autonomous_nav:ro \
  ros2-jazzy-dev:codex-check -lc 'set -e; source /opt/ros/jazzy/setup.bash;
  cd /tmp/ws; colcon build --packages-select turtleboot3_autonomous_nav;
  source install/setup.bash;
  pytest src/turtleboot3_autonomous_nav/test -q -p no:cacheprovider'
```

Build succeeded; **40 passed**. The source mount was read-only and build/install
outputs stayed in the disposable container. An earlier non-root container build
failed on a container temporary-directory permission; the final command above
resolved that without modifying workspace permissions.

```text
PYTHONPATH=ros2_ws/src/turtleboot3_autonomous_nav python3 -m pytest \
  ros2_ws/src/turtleboot3_autonomous_nav/test/test_dqn_training.py -q
```

Host pure trainer suite: **18 passed**. The host lacks NumPy/ROS, so mapper and
real ROS tests ran in the container. `black --check` on all six changed Python
files and `git diff --check` also passed. No Gazebo world was launched in this
follow-up; the full launch/episode checkpoint exercise remains Task 6 work.

Commit: `1aca128` — `fix: reject queued ROS publications across episode reset`.
