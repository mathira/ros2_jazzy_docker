# TurtleBot3 Autonomous Exploration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `turtleboot3_autonomous_nav`: a ROS 2 Jazzy Python package that autonomously maps and explores `turtlebot3_dqn_stage4` with odometry, LiDAR, a DQN policy, and a safety controller.

**Architecture:** A mapper transforms `/scan` plus `/odom` into `/coverage_map` in `odom`. The observation builder converts the map and LiDAR to a fixed vector; DQN selects discrete actions, while the safety controller is the exclusive `/cmd_vel` publisher. Training reuses mission components but alone updates model weights.

**Tech Stack:** ROS 2 Jazzy, rclpy, Gazebo Harmonic/ros_gz, TurtleBot3, NumPy, PyTorch, pytest, RViz.

**Spec:** `docs/superpowers/specs/2026-09-04-turtlebot3-autonomous-exploration-design.md`

## Global Constraints

- Create exactly `ros2_ws/src/turtleboot3_autonomous_nav`; preserve existing Stage-project files and changes.
- Include official `turtlebot3_gazebo/turtlebot3_dqn_stage4.launch.py` as simulation entry point.
- Inputs are `/scan`, `/odom`, TF; only `safe_motion_controller` publishes `/cmd_vel`.
- Do not include Nav2, `map_server`, AMCL, SLAM Toolbox, or a prebuilt map.
- Publish `nav_msgs/OccupancyGrid` at `/coverage_map` in `odom`.
- Training runs headless by default, resets Gazebo each episode, and persists the best model plus JSON metrics.
- Mission loads a fixed model and requires no teleoperation.

---

## File Structure

```text
.devcontainer/Dockerfile
ros2_ws/src/turtleboot3_autonomous_nav/
├── config/{exploration,training}.yaml
├── launch/{mission,training}.launch.py
├── models/README.md
├── resource/turtleboot3_autonomous_nav
├── rviz/exploration.rviz
├── test/{test_grid_mapping,test_observation_and_control,test_dqn_training,test_launch_assets}.py
├── turtleboot3_autonomous_nav/{grid_mapping,coverage_mapper,observation,observation_builder,control,safe_motion_controller,dqn,dqn_explorer,dqn_trainer,metrics}.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

### Task 1: Scaffold the package and dependency contract

**Files:** Modify `.devcontainer/Dockerfile`. Create the package metadata, resource marker, empty module, `config/exploration.yaml`, `config/training.yaml`, and `test/test_launch_assets.py`.

**Interfaces:** Console scripts: `coverage_mapper`, `observation_builder`, `safe_motion_controller`, `dqn_explorer`, `dqn_trainer`.

- [ ] **Step 1: Write the failing dependency test.**

```python
def test_package_declares_runtime_dependencies():
    xml = (PACKAGE_ROOT / 'package.xml').read_text()
    for name in ('rclpy', 'sensor_msgs', 'nav_msgs', 'geometry_msgs', 'tf2_ros', 'ros_gz_interfaces'):
        assert f'<exec_depend>{name}</exec_depend>' in xml
```

- [ ] **Step 2: Run it and confirm failure.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_launch_assets.py -v`

Expected: FAIL because the package is absent.

- [ ] **Step 3: Implement minimal package metadata.**

Add an `ament_python` package, five console scripts, and `data_files` for launch/config/RViz/models. Add `python3-torch` to the current apt dependency block in `.devcontainer/Dockerfile`; use `numpy` as the only Python runtime dependency in `setup.py`.

- [ ] **Step 4: Build and test.**

Run: `cd ros2_ws && colcon build --packages-select turtleboot3_autonomous_nav && source install/setup.bash && pytest src/turtleboot3_autonomous_nav/test/test_launch_assets.py -v`

Expected: PASS.

- [ ] **Step 5: Commit.**

Run: `git add .devcontainer/Dockerfile ros2_ws/src/turtleboot3_autonomous_nav && git commit -m "feat: scaffold TurtleBot3 exploration package"`

### Task 2: Implement odometric occupancy mapping

**Files:** Create `grid_mapping.py`, `coverage_mapper.py`, `test/test_grid_mapping.py`; modify `config/exploration.yaml`.

**Interfaces:** `OccupancyGridModel.update_scan(pose: tuple[float, float, float], ranges: np.ndarray, angle_min: float, angle_increment: float, range_max: float) -> int`; `to_message(stamp, frame_id='odom') -> OccupancyGrid`; `coverage_fraction() -> float`. Node consumes `/scan` and `/odom`; publishes `/coverage_map` and `/coverage_metrics`.

- [ ] **Step 1: Write the failing ray-tracing test.**

```python
def test_hit_marks_free_cells_then_occupied_endpoint():
    grid = OccupancyGridModel(20, 20, 1.0, (-10.0, -10.0))
    grid.update_scan((0.0, 0.0, 0.0), np.array([3.0]), 0.0, 1.0, 8.0)
    assert grid.value_at(1.0, 0.0) == 0
    assert grid.value_at(3.0, 0.0) == 100
```

- [ ] **Step 2: Run it and confirm failure.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_grid_mapping.py::test_hit_marks_free_cells_then_occupied_endpoint -v`

Expected: FAIL with import error.

- [ ] **Step 3: Implement pure mapping, then ROS wrapper.**

Implement bounded Bresenham traversal, `-1/0/100` grid values, finite-range filtering, max-range free-space tracing, occupancy evidence threshold, and monotonic coverage. In the node, synchronize each scan with the newest odometry/TF pose and do not publish until both exist.

- [ ] **Step 4: Test edge cases.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_grid_mapping.py -v`

Expected: PASS for endpoint, max-range, invalid range, bounds, and coverage tests.

- [ ] **Step 5: Commit.**

Run: `git add ros2_ws/src/turtleboot3_autonomous_nav && git commit -m "feat: map explored space from lidar and odometry"`

### Task 3: Build observations and safe controller

**Files:** Create `observation.py`, `observation_builder.py`, `control.py`, `safe_motion_controller.py`, `test/test_observation_and_control.py`.

**Interfaces:** `build_observation(scan_ranges, local_grid, linear_velocity, angular_velocity) -> np.ndarray`; `safe_twist(action: int, sector_ranges: np.ndarray, stalled: bool, config: ControlConfig) -> TwistDecision`. Builder publishes `/dqn_observation` (`Float32MultiArray`); controller consumes action + scan and publishes `/cmd_vel`.

- [ ] **Step 1: Write the failing stop test.**

```python
def test_front_obstacle_overrides_forward_action():
    result = safe_twist(FORWARD, np.array([0.15, 2.0, 2.0]), False, ControlConfig(stop_distance=0.20))
    assert result.linear_x == 0.0
    assert result.intervention is True
```

- [ ] **Step 2: Run it and confirm failure.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_observation_and_control.py::test_front_obstacle_overrides_forward_action -v`

Expected: FAIL with import error.

- [ ] **Step 3: Implement pure behavior and node wrappers.**

Reduce LiDAR to 12 minima, map to eight directional unknown-area gains, and include a fixed local grid patch and velocity. Define `FORWARD`, `SOFT_LEFT`, `SOFT_RIGHT`, `LEFT`, `RIGHT`, `RECOVER`. Clamp speeds, turn to clearer side at emergency stop, recover after stalled progress, and publish zero for stale data.

- [ ] **Step 4: Run all component tests.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_observation_and_control.py -v`

Expected: PASS for fixed observation shape, stop, escape direction, stale sensor and recovery.

- [ ] **Step 5: Commit.**

Run: `git add ros2_ws/src/turtleboot3_autonomous_nav && git commit -m "feat: add observations and safe motion control"`

### Task 4: Implement DQN inference, replay and checkpointing

**Files:** Create `dqn.py`, `dqn_explorer.py`, `metrics.py`, `test/test_dqn_training.py`.

**Interfaces:** `DQNPolicy(observation_size: int, action_count: int)`; `ReplayBuffer.add(state, action, reward, next_state, done)`; `save_checkpoint(path, policy, config, metrics)`; `load_checkpoint(path) -> tuple[DQNPolicy, dict, dict]`. Explorer consumes `/dqn_observation`; publishes `Int32` `/exploration_action`.

- [ ] **Step 1: Write the failing checkpoint test.**

```python
def test_checkpoint_round_trip_preserves_action_space(tmp_path):
    policy = DQNPolicy(observation_size=10, action_count=6)
    save_checkpoint(tmp_path / 'best.pt', policy, {'epsilon': 0.0}, {'coverage': 0.5})
    loaded, _, metrics = load_checkpoint(tmp_path / 'best.pt')
    assert loaded.action_count == 6
    assert metrics['coverage'] == 0.5
```

- [ ] **Step 2: Run it and confirm failure.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_dqn_training.py::test_checkpoint_round_trip_preserves_action_space -v`

Expected: FAIL with import error.

- [ ] **Step 3: Implement DQN and inference.**

Use a two-hidden-layer fully connected network, bounded replay buffer, target-network copying, epsilon-greedy training selection and greedy inference. Checkpoint weights, dimensions, config and metrics; reject a dimensional mismatch before inference.

- [ ] **Step 4: Run DQN tests.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_dqn_training.py -v`

Expected: PASS for replay capacity, epsilon behavior, checkpoint round trip and mismatch rejection.

- [ ] **Step 5: Commit.**

Run: `git add ros2_ws/src/turtleboot3_autonomous_nav && git commit -m "feat: add DQN exploration policy"`

### Task 5: Train across Gazebo episodes

**Files:** Create `dqn_trainer.py`; modify `config/training.yaml` and `test/test_dqn_training.py`.

**Interfaces:** Consumes observation, coverage metrics, controller safety events, and Gazebo `ControlWorld`; writes `models/best.pt` and `models/best.metrics.json`; publishes `/training_metrics`.

- [ ] **Step 1: Write the failing reward test.**

```python
def test_reward_favors_new_coverage_and_penalizes_intervention():
    assert episode_reward(12, True, False, False) > 0
    assert episode_reward(0, False, True, True) < 0
```

- [ ] **Step 2: Run it and confirm failure.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_dqn_training.py::test_reward_favors_new_coverage_and_penalizes_intervention -v`

Expected: FAIL with missing `episode_reward`.

- [ ] **Step 3: Implement trainer.**

Reward newly observed cells plus frontier arrival; subtract per-step, no-progress, intervention and recovery costs. End episodes by max steps, target coverage or stall count. Reset `/world/dqn/control` through `ros_gz_interfaces/srv/ControlWorld` with reset-all; wait for fresh odometry and scan before next episode. Evaluate periodically with epsilon zero and save only improved mean-coverage checkpoints.

- [ ] **Step 4: Verify trainer.**

Run: `cd ros2_ws && colcon build --packages-select turtleboot3_autonomous_nav && source install/setup.bash && pytest src/turtleboot3_autonomous_nav/test/test_dqn_training.py -v`

Expected: PASS. Run one short headless episode; `models/best.pt` and `models/best.metrics.json` exist.

- [ ] **Step 5: Commit.**

Run: `git add ros2_ws/src/turtleboot3_autonomous_nav && git commit -m "feat: train exploration DQN in Gazebo"`

### Task 6: Launch, RViz, README and integration

**Files:** Create `launch/mission.launch.py`, `launch/training.launch.py`, `rviz/exploration.rviz`, `models/README.md`, `README.md`; modify `test/test_launch_assets.py`.

**Interfaces:** Mission accepts `model_path`, `use_rviz`, `use_sim_time`; training accepts `episodes`, `use_gui`, `use_sim_time`. Both include official stage4 launcher.

- [ ] **Step 1: Write the failing launch-contract test.**

```python
def test_mission_uses_stage4_without_nav_or_slam():
    source = (PACKAGE_ROOT / 'launch' / 'mission.launch.py').read_text()
    assert 'turtlebot3_dqn_stage4.launch.py' in source
    assert 'nav2' not in source.lower()
    assert 'slam' not in source.lower()
```

- [ ] **Step 2: Run it and confirm failure.**

Run: `cd ros2_ws/src/turtleboot3_autonomous_nav && pytest test/test_launch_assets.py -v`

Expected: FAIL because launcher is absent.

- [ ] **Step 3: Implement launcher and user artifacts.**

Use `IncludeLaunchDescription` for the official world. Mission starts mapper, builder, explorer, controller and optional RViz. Training starts mapper, builder, controller, trainer, and disables GUI by default; it must not start explorer. Configure RViz for map, scan, odom, cmd_vel and metrics. Document source dependencies, container rebuild, build, training command, mission command, output artifacts, and no-manual-control demonstration.

- [ ] **Step 4: Execute tests and smoke integration.**

Run: `cd ros2_ws && colcon build --packages-select turtleboot3_autonomous_nav && source install/setup.bash && pytest src/turtleboot3_autonomous_nav/test -v`

Expected: PASS. Launch mission headlessly with a test model; verify `/coverage_map`, `/exploration_action`, `/cmd_vel` publish and `ros2 node info` shows controller as sole `/cmd_vel` publisher.

- [ ] **Step 5: Commit.**

Run: `git add ros2_ws/src/turtleboot3_autonomous_nav && git commit -m "feat: launch and document DQN exploration mission"`

## Final Verification

- [ ] Run `git diff --check`.
- [ ] Run `cd ros2_ws && colcon build --packages-select turtleboot3_autonomous_nav`.
- [ ] Run `cd ros2_ws && source install/setup.bash && pytest src/turtleboot3_autonomous_nav/test -v`.
- [ ] Run short headless training; verify checkpoint and JSON metrics.
- [ ] Run headless mission from that checkpoint; verify growing `/coverage_map`, autonomous `/cmd_vel`, and no Nav2/SLAM/AMCL/map-server nodes.
