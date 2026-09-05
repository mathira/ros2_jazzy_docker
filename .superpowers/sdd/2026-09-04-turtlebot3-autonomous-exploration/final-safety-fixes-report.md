# Final safety review fixes

Implemented in `ros2_ws/src/turtleboot3_autonomous_nav`, following systematic
debugging and TDD. The original failures were reproduced before implementation:
missing odometry and negative simulated ages incorrectly passed the controller
freshness check; lifecycle/reset services and mission termination were absent;
and an empty model path silently created an untrained policy.

## Changes

- The controller starts disabled and exposes the acknowledged
  `/safe_motion_controller/enable` (`std_srvs/SetBool`) service. Disable and enable
  both publish zero, clear cached inputs/progress/recovery, and establish a fresh
  DDS publication cutoff. Commands require newer scan, odometry, and action
  publications. Queued pre-reset inputs cannot restore motion.
- Scan and odometry freshness are required, negative simulated ages fail closed,
  and a steady-clock control timer also enforces wall-clock input age. Sensor
  failure remains visible through the safety-intervention output.
- Stall recovery is bounded by `recovery_duration` (default two simulated
  seconds), then restores a progress interval for normal policy actions.
  Repeated policy recovery actions stop after the recovery interval until the
  policy changes action.
- The observation builder has an acknowledged reset service, clears every
  cached input, filters DDS publications against its reset cutoff, and includes
  an episode epoch in the observation layout. The trainer rejects observations
  carrying a previous epoch even when their publication time is recent.
- Trainer reset ordering is controller disable ACK, world reset ACK, mapper
  reset ACK, builder reset ACK, controller enable ACK, fresh scan/odometry, and
  a current-epoch observation. The existing mapper/trainer DDS publication
  barrier remains enforced; simulated sensor headers never replace publication
  provenance.
- Service discovery and responses, post-reset sensors, and observations have
  steady/wall-clock deadlines. Failed or timed-out reset transactions suppress
  actions, request an acknowledged stop, and terminate unsuccessfully. Training
  completion requests an acknowledged stop before normal exit.
- Mission inference requires a nonempty valid model path, counts actions, and
  stops on target coverage or the configured action limit. Both terminal paths
  disable the controller and suppress subsequent actions. Mission launch exposes
  `max_steps` and `target_coverage`; package documentation describes the lifecycle
  services, limits, and deadline behavior.

## Verification

Executed in the existing `confident_aryabhata` Jazzy container:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
colcon build --packages-select turtleboot3_autonomous_nav
colcon test --packages-select turtleboot3_autonomous_nav --event-handlers console_direct+
colcon test-result --test-result-base build/turtleboot3_autonomous_nav --verbose
```

Results: build passed; **68 tests, zero errors, zero failures, zero skipped**.
Direct pytest of the package also passed all 68 tests. Coverage includes real ROS
message/controller behavior, controlled asynchronous service transport, delayed
old-publication and wrong-epoch rejection, all reset response deadlines, missing
odometry and frozen/rewound clocks, bounded recovery, launch parameter forwarding,
mission zero commands, training stop/failure behavior, and absent model rejection.
The original real-DDS queued-publication test still passes.

Installed command-line smoke checks ran in isolated `ROS_DOMAIN_ID=91`:

- `ros2 run turtleboot3_autonomous_nav dqn_trainer --ros-args -p reset_timeout_seconds:=0.2`
  with no controller service: bounded discovery timeout, bounded stop-ACK
  timeout, and exit status 1 with an explicit training-aborted error.
- `ros2 run turtleboot3_autonomous_nav dqn_explorer` with no model: exit status 1
  with the explicit required-checkpoint error before controller activation.

`git diff --check` passed. No unrelated Stage, devcontainer, or root README
changes were staged or modified by this task.

## Remaining external limitation

A fresh full-Gazebo training campaign was not part of this verification. The
previously documented Gazebo 8.11 DiffDrive odometry loss after reset.all remains
an external limitation; these fixes convert that missing-input wait into a
bounded, explicit failure. No mapper freshness barrier or full-reset requirement
was relaxed to bypass it. These results establish safety/lifecycle behavior,
not a trained policy's exploration quality.
