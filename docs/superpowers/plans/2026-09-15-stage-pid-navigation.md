# Stage PID Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an independent ROS 2 Jazzy package that drives a Stage differential robot autonomously to a configurable `(x, y)` goal using PID heading control and LiDAR collision avoidance.

**Architecture:** `stage_pid_navigation` is an `ament_python` package. Pure control logic in `control.py` is independently unit-tested; `pid_navigator.py` adapts odometry and laser callbacks to that logic and publishes `Twist`. A launch file supplies goal, topic and safety parameters without launching the simulator.

**Tech Stack:** Python 3, ROS 2 Jazzy (`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `launch_ros`), pytest, colcon.

**Spec:** `docs/superpowers/specs/2026-09-15-stage-pid-navigation-design.md`

## Global Constraints

- Create only `ros2_ws/src/stage_pid_navigation`; do not modify an existing ROS package.
- Default interfaces must be `/odom` (`nav_msgs/msg/Odometry`), `/base_scan` (`sensor_msgs/msg/LaserScan`) and `/cmd_vel` (`geometry_msgs/msg/Twist`).
- Topic parameters must permit `/robot_0/odom`, `/robot_0/base_scan` and `/robot_0/cmd_vel`.
- Default safety requires an initial LiDAR reading before motion.
- The normal Stage configuration uses unstamped `Twist`; do not use Ackermann or `TwistStamped`.

---

### Task 1: Scaffold the isolated ROS 2 package and validate its public launcher

**Files:**
- Create: `ros2_ws/src/stage_pid_navigation/package.xml`
- Create: `ros2_ws/src/stage_pid_navigation/setup.py`
- Create: `ros2_ws/src/stage_pid_navigation/setup.cfg`
- Create: `ros2_ws/src/stage_pid_navigation/resource/stage_pid_navigation`
- Create: `ros2_ws/src/stage_pid_navigation/stage_pid_navigation/__init__.py`
- Create: `ros2_ws/src/stage_pid_navigation/stage_pid_navigation/launch/pid_navigation.launch.py`
- Create: `ros2_ws/src/stage_pid_navigation/test/test_launch_description.py`

**Interfaces:**
- Produces `ros2 launch stage_pid_navigation pid_navigation.launch.py goal_x:=2.0 goal_y:=1.5`.
- Produces launch arguments `goal_x`, `goal_y`, `odom_topic`, `scan_topic`, `cmd_vel_topic`, `control_rate`, `kp`, `ki`, `kd`, `integral_limit`, `max_linear_speed`, `max_angular_speed`, `heading_stop_threshold`, `goal_tolerance`, `slowdown_distance`, `stop_distance`, `front_sector_angle` and `require_scan`.

- [ ] **Step 1: Write the failing launch contract test**

```python
def test_launch_declares_stage_topics_and_goal_arguments():
    description = generate_launch_description()
    names = {action.name for action in description.entities if hasattr(action, "name")}
    assert {"goal_x", "goal_y", "odom_topic", "scan_topic", "cmd_vel_topic"} <= names
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd ros2_ws && python3 -m pytest src/stage_pid_navigation/test/test_launch_description.py -v`

Expected: FAIL because the package and `pid_navigation.launch` module do not exist.

- [ ] **Step 3: Add the minimal ament_python metadata and launch file**

Use `ament_python`, declare runtime dependencies on `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs` and `launch_ros`, install the package resource and install `launch/*.launch.py`. The launch file must construct a `launch_ros.actions.Node` for executable `pid_navigator` and pass every declared argument as a parameter. Default topic values must be `/odom`, `/base_scan` and `/cmd_vel`.

- [ ] **Step 4: Run the launch contract test to verify it passes**

Run: `cd ros2_ws && python3 -m pytest src/stage_pid_navigation/test/test_launch_description.py -v`

Expected: PASS.

- [ ] **Step 5: Commit the scaffold**

```bash
git add ros2_ws/src/stage_pid_navigation
git commit -m "feat: scaffold Stage PID navigation package"
```

### Task 2: Implement and unit-test the pure navigation and safety controller

**Files:**
- Create: `ros2_ws/src/stage_pid_navigation/stage_pid_navigation/control.py`
- Create: `ros2_ws/src/stage_pid_navigation/test/test_control.py`

**Interfaces:**
- Produces `normalize_angle(angle: float) -> float` in `[-pi, pi]`.
- Produces `PidController(kp, ki, kd, integral_limit).update(error, dt) -> float`.
- Produces `compute_command(pose, goal, scan, config, pid, dt) -> Command`, where `Command` has `linear`, `angular`, `reached_goal` and `blocked`.
- Consumes `Pose2D(x, y, yaw)`, `ScanSummary(front, left, right)` and `NavigationConfig`.

- [ ] **Step 1: Write failing tests for the math and the goal controller**

```python
def test_normalize_angle_wraps_across_pi():
    assert normalize_angle(3.5) < 0.0

def test_pid_clamps_integral_accumulation():
    pid = PidController(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5)
    assert pid.update(error=2.0, dt=1.0) == 0.5

def test_controller_stops_inside_goal_tolerance():
    command = compute_command(Pose2D(1.0, 1.0, 0.0), (1.05, 1.0), ScanSummary.clear(), config, pid, 0.1)
    assert command.reached_goal and command.linear == 0.0 and command.angular == 0.0

def test_controller_does_not_advance_until_heading_is_aligned():
    command = compute_command(Pose2D(0.0, 0.0, 3.14), (1.0, 0.0), ScanSummary.clear(), config, pid, 0.1)
    assert command.linear == 0.0 and command.angular < 0.0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ros2_ws && python3 -m pytest src/stage_pid_navigation/test/test_control.py -v`

Expected: FAIL because `control.py` is absent.

- [ ] **Step 3: Implement the smallest controller that satisfies those tests**

Implement dataclasses for pose, scan summary, configuration and command. Compute the Euclidean goal distance and heading error; return a zero command inside `goal_tolerance`; pass heading error through PID; clamp angular speed; set linear speed to zero when `abs(heading_error) > heading_stop_threshold`, otherwise make it proportional to distance and cap it at `max_linear_speed`. Clamp PID integral before multiplying by `ki`, skip derivative when no valid prior error exists, and treat non-positive `dt` as zero integral/derivative update.

- [ ] **Step 4: Add failing obstacle-safety tests**

```python
def test_obstacle_inside_stop_distance_blocks_forward_motion_and_turns_to_clearer_side():
    scan = ScanSummary(front=0.2, left=0.9, right=0.4)
    command = compute_command(Pose2D(0.0, 0.0, 0.0), (2.0, 0.0), scan, config, pid, 0.1)
    assert command.blocked and command.linear == 0.0 and command.angular > 0.0

def test_obstacle_inside_slowdown_distance_reduces_linear_speed():
    clear = compute_command(Pose2D(0.0, 0.0, 0.0), (2.0, 0.0), ScanSummary.clear(), config, pid, 0.1)
    slowed = compute_command(Pose2D(0.0, 0.0, 0.0), (2.0, 0.0), ScanSummary(0.6, 1.0, 1.0), config, pid, 0.1)
    assert 0.0 < slowed.linear < clear.linear

def test_scan_summary_ignores_nan_infinity_and_ranges_outside_front_sector():
    assert summarize_scan(ranges, angle_min, angle_increment, front_sector_angle).front == 0.8
```

- [ ] **Step 5: Run obstacle tests to verify they fail**

Run: `cd ros2_ws && python3 -m pytest src/stage_pid_navigation/test/test_control.py -v`

Expected: FAIL on missing safety behavior and scan extraction.

- [ ] **Step 6: Implement LiDAR summarization and safety override**

Consider only finite ranges within the requested forward/left/right angular sectors. Below `stop_distance`, emit no linear velocity and turn toward the larger finite side clearance; deterministic ties turn left. Between `stop_distance` and `slowdown_distance`, multiply linear velocity by `(front - stop_distance) / (slowdown_distance - stop_distance)`. At or beyond the slowdown distance, preserve the nominal command.

- [ ] **Step 7: Run the controller tests to verify they pass**

Run: `cd ros2_ws && python3 -m pytest src/stage_pid_navigation/test/test_control.py -v`

Expected: PASS.

- [ ] **Step 8: Commit the controller**

```bash
git add ros2_ws/src/stage_pid_navigation/stage_pid_navigation/control.py ros2_ws/src/stage_pid_navigation/test/test_control.py
git commit -m "feat: add PID goal controller with LiDAR safety"
```

### Task 3: Integrate the controller into the ROS node and document operation

**Files:**
- Create: `ros2_ws/src/stage_pid_navigation/stage_pid_navigation/pid_navigator.py`
- Modify: `ros2_ws/src/stage_pid_navigation/setup.py`
- Create: `ros2_ws/src/stage_pid_navigation/README.md`
- Create: `ros2_ws/src/stage_pid_navigation/test/test_pid_navigator.py`

**Interfaces:**
- Consumes `PidController`, `NavigationConfig`, `Pose2D`, `ScanSummary`, `summarize_scan` and `compute_command` from `control.py`.
- Produces executable `pid_navigator` and publishes `geometry_msgs/msg/Twist` on parameter `cmd_vel_topic`.

- [ ] **Step 1: Write the failing ROS adapter tests**

```python
def test_odometry_callback_converts_quaternion_to_planar_pose():
    navigator = make_navigator_for_test()
    navigator._on_odom(odometry_with_yaw(math.pi / 2))
    assert navigator._pose == Pose2D(2.0, -1.0, pytest.approx(math.pi / 2))

def test_timer_publishes_stop_without_pose_or_required_scan():
    navigator = make_navigator_for_test(require_scan=True)
    navigator._on_control_timer()
    assert last_published_twist(navigator) == Twist()

def test_timer_converts_command_to_twist():
    navigator = ready_navigator_for_test(command=Command(0.2, -0.4, False, False))
    navigator._on_control_timer()
    assert last_published_twist(navigator).linear.x == 0.2
    assert last_published_twist(navigator).angular.z == -0.4
```

- [ ] **Step 2: Run the adapter tests to verify they fail**

Run: `cd ros2_ws && python3 -m pytest src/stage_pid_navigation/test/test_pid_navigator.py -v`

Expected: FAIL because `pid_navigator.py` and the console executable do not exist.

- [ ] **Step 3: Implement the node and executable entry point**

Declare all launch parameters in `PidNavigator`. Subscribe to configured `Odometry` and `LaserScan` topics, convert quaternion yaw with the standard `atan2` expression, and use a fixed-rate timer based on `control_rate`. Publish zero `Twist` before a pose exists, before a required scan exists, after the goal is reached and from an `on_shutdown` callback. Add `pid_navigator = stage_pid_navigation.pid_navigator:main` to `console_scripts`.

- [ ] **Step 4: Run the adapter tests to verify they pass**

Run: `cd ros2_ws && python3 -m pytest src/stage_pid_navigation/test/test_pid_navigator.py -v`

Expected: PASS.

- [ ] **Step 5: Write the operational README**

Include exact build and sourcing commands, how to launch single-robot Stage (`world:=cave`), the default navigation invocation, the multi-robot remapping invocation, parameter descriptions and an explicit limitation: a local LiDAR reaction cannot guarantee reaching a goal behind a non-traversable obstacle without global planning.

- [ ] **Step 6: Build and run package tests**

Run:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stage_pid_navigation
source install/setup.bash
python3 -m pytest src/stage_pid_navigation/test -v
```

Expected: build succeeds and every package test passes.

- [ ] **Step 7: Perform Stage integration smoke test**

In one terminal run `ros2 launch stage_ros2 stage.launch.py world:=cave`. In another sourced terminal run `ros2 launch stage_pid_navigation pid_navigation.launch.py goal_x:=-6.0 goal_y:=-5.5`. Confirm `/cmd_vel` is published, the robot stops within `goal_tolerance`, and no contact occurs for this free path.

- [ ] **Step 8: Commit the integration and documentation**

```bash
git add ros2_ws/src/stage_pid_navigation
git commit -m "feat: integrate PID navigation node for Stage"
```

## Plan self-review

- Spec coverage: Task 1 supplies the package, launch interface and confirmed Stage topics. Task 2 supplies PID behavior, tolerance, saturation, LiDAR processing and collision reaction. Task 3 supplies ROS integration, safe startup/shutdown, build, tests and user-facing operation instructions.
- Placeholder scan: no deferred implementation markers are present.
- Type consistency: Task 2 defines the pure inputs/outputs consumed by Task 3; task names and entry point are consistently `pid_navigator`.
