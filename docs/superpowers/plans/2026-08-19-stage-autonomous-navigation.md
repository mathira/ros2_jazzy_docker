# Stage Autonomous Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `stage_autonomous_nav`, a ROS 2 Jazzy package that launches Stage `cave`, Nav2 and RViz so a user can navigate the simulated Pioneer with RViz 2D Goal Pose.

**Architecture:** `stage_ros2` supplies `/odom`, `/ground_truth`, `/base_scan`, TF and `/cmd_vel`. A small package node derives the global `map -> odom` transform from Stage ground truth, while Nav2 uses a static map for global planning and `/base_scan` for local obstacle avoidance. The launch owns all orchestration and exposes frame and topic names as arguments.

**Tech Stack:** ROS 2 Jazzy, Python/rclpy, launch, tf2_ros, Nav2, RViz2, `stage_ros2`, Stage, pytest/ament.

**Spec:** `docs/superpowers/specs/2026-08-19-stage-autonomous-navigation-design.md`

## Global Constraints

- Target ROS distribution: Jazzy.
- Use the `jazzy` branch of `https://github.com/tuw-robotics/stage_ros2`.
- The first supported world is `cave`, launched with `enforce_prefixes:=false` and `one_tf_tree:=true`.
- Navigation uses RViz **2D Goal Pose**; no custom goal UI, AMCL, SLAM or multirobot support.
- Use simulation time everywhere.
- Default Stage interfaces are `/odom`, `/ground_truth`, `/base_scan`, `/cmd_vel`, `odom` and `base_link`.
- `ground_truth_localizer` must publish `map -> odom` as `T_map_base * inverse(T_odom_base)`; it must never publish `odom -> base_link`.
- The dynamic block at `(5, 4)` stays out of the static map and must be handled by Nav2's laser-based local costmap.
- Preserve unrelated changes already present in `.devcontainer/Dockerfile`, `.devcontainer/devcontainer.json`, `README.md` and `.DS_Store`.

---

## Planned file structure

```text
ros2_ws/
├── stage_nav.repos
└── src/stage_autonomous_nav/
    ├── config/nav2_cave.yaml
    ├── launch/cave_navigation.launch.py
    ├── maps/cave.yaml
    ├── maps/cave.pgm
    ├── resource/stage_autonomous_nav
    ├── rviz/cave_navigation.rviz
    ├── stage_autonomous_nav/__init__.py
    ├── stage_autonomous_nav/ground_truth_localizer.py
    ├── stage_autonomous_nav/launch/__init__.py
    ├── stage_autonomous_nav/launch/cave_navigation.py
    ├── stage_autonomous_nav/transform_math.py
    ├── test/test_ground_truth_localizer.py
    ├── test/test_launch_description.py
    ├── package.xml
    ├── setup.cfg
    ├── setup.py
    └── README.md
```

### Task 1: Reproducible Stage/Nav2 workspace prerequisites

**Files:**
- Create: `ros2_ws/stage_nav.repos`
- Modify: `.devcontainer/Dockerfile`
- Modify: `README.md`

**Interfaces:**
- Consumes: a ROS 2 Jazzy container and `vcs import`.
- Produces: sources named `Stage` and `stage_ros2` under `ros2_ws/src`, plus Nav2 and Stage build prerequisites in the image.

- [ ] **Step 1: Add the external source manifest**

Create `ros2_ws/stage_nav.repos` with both repositories and explicit versions:

```yaml
repositories:
  Stage:
    type: git
    url: https://github.com/tuw-robotics/Stage.git
    version: jazzy
  stage_ros2:
    type: git
    url: https://github.com/tuw-robotics/stage_ros2.git
    version: jazzy
```

- [ ] **Step 2: Add runtime packages to the dev container**

In the existing ROS package-install `RUN apt-get ...` block of `.devcontainer/Dockerfile`, add these packages alongside the current simulation dependencies:

```dockerfile
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-ackermann-msgs \
    python3-pil \
```

Do not alter existing package lines or the current VNC setup.

- [ ] **Step 3: Document import and build commands**

Add a `Stage + Nav2 autonomous navigation` section to `README.md` with exactly these preparation commands, run inside the container before building the workspace:

```bash
cd /ros2_ws
vcs import src < stage_nav.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

State that a rebuild of the dev-container image is required after Dockerfile changes.

- [ ] **Step 4: Verify dependency metadata is parseable**

Run: `vcs validate < ros2_ws/stage_nav.repos`

Expected: exit status 0 and both repositories reported as valid.

- [ ] **Step 5: Commit only prerequisite files**

```bash
git add ros2_ws/stage_nav.repos .devcontainer/Dockerfile README.md
git commit -m "chore: add Stage and Nav2 workspace prerequisites"
```

If either existing modified file contains unrelated user changes, stage only the exact hunks added for this task with `git add -p`; never overwrite or commit the unrelated hunks.

### Task 2: Scaffold the package and implement transform math

**Files:**
- Create: `ros2_ws/src/stage_autonomous_nav/package.xml`
- Create: `ros2_ws/src/stage_autonomous_nav/setup.py`
- Create: `ros2_ws/src/stage_autonomous_nav/setup.cfg`
- Create: `ros2_ws/src/stage_autonomous_nav/resource/stage_autonomous_nav`
- Create: `ros2_ws/src/stage_autonomous_nav/stage_autonomous_nav/__init__.py`
- Create: `ros2_ws/src/stage_autonomous_nav/stage_autonomous_nav/transform_math.py`
- Create: `ros2_ws/src/stage_autonomous_nav/test/test_ground_truth_localizer.py`

**Interfaces:**
- Consumes: two planar poses `(x, y, yaw)` where the first is `map -> base_link` and the second is `odom -> base_link`.
- Produces: `map_to_odom(map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw) -> tuple[float, float, float]` representing `map -> odom`.

- [ ] **Step 1: Write the failing transform-math tests**

Create `test/test_ground_truth_localizer.py`:

```python
from math import isclose, pi

from stage_autonomous_nav.transform_math import map_to_odom


def test_map_to_odom_is_identity_for_equal_base_poses():
    assert map_to_odom(2.0, -1.0, 0.4, 2.0, -1.0, 0.4) == (0.0, 0.0, 0.0)


def test_map_to_odom_composes_global_pose_with_inverse_odom_pose():
    x, y, yaw = map_to_odom(10.0, 5.0, pi / 2.0, 1.0, 0.0, 0.0)
    assert isclose(x, 10.0, abs_tol=1e-9)
    assert isclose(y, 4.0, abs_tol=1e-9)
    assert isclose(yaw, pi / 2.0, abs_tol=1e-9)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd ros2_ws && pytest src/stage_autonomous_nav/test/test_ground_truth_localizer.py -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'stage_autonomous_nav'`.

- [ ] **Step 3: Create the package manifest and Python installation metadata**

Use `ament_python`. Declare `rclpy`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `stage_ros2`, `nav2_bringup`, `nav2_map_server`, `rviz2` and `python3-pytest` test dependency. Install launch, config, maps and RViz resources in `setup.py`; define the console script `ground_truth_localizer = stage_autonomous_nav.ground_truth_localizer:main`.

- [ ] **Step 4: Implement the pure planar composition function**

Create `transform_math.py` with no ROS imports:

```python
from math import cos, sin


def map_to_odom(map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw):
    yaw = map_yaw - odom_yaw
    return (
        map_x - cos(yaw) * odom_x + sin(yaw) * odom_y,
        map_y - sin(yaw) * odom_x - cos(yaw) * odom_y,
        yaw,
    )
```

- [ ] **Step 5: Run the unit test**

Run: `cd ros2_ws && PYTHONPATH=src/stage_autonomous_nav pytest src/stage_autonomous_nav/test/test_ground_truth_localizer.py -v`

Expected: PASS, 2 tests.

- [ ] **Step 6: Commit the package foundation**

```bash
git add ros2_ws/src/stage_autonomous_nav
git commit -m "feat: scaffold Stage autonomous navigation package"
```

### Task 3: Publish `map -> odom` from Stage ground truth

**Files:**
- Create: `ros2_ws/src/stage_autonomous_nav/stage_autonomous_nav/ground_truth_localizer.py`
- Modify: `ros2_ws/src/stage_autonomous_nav/test/test_ground_truth_localizer.py`

**Interfaces:**
- Consumes: `nav_msgs/msg/Odometry` from `/ground_truth` and TF `odom -> base_link`.
- Produces: a `geometry_msgs/msg/TransformStamped` broadcast on `/tf` whose header frame is `map` and child frame is `odom`.
- Parameters: `use_sim_time` (bool), `ground_truth_topic` (string, default `/ground_truth`), `map_frame` (`map`), `odom_frame` (`odom`), `base_frame` (`base_link`), `publish_rate_hz` (`20.0`).

- [ ] **Step 1: Extend tests for conversion helpers**

Add tests that call module-level helpers `yaw_from_quaternion(x, y, z, w)` and `quaternion_from_yaw(yaw)`. Assert yaw `pi / 2` becomes `(z=sqrt(0.5), w=sqrt(0.5))` and converts back to `pi / 2` with `abs_tol=1e-9`.

- [ ] **Step 2: Run tests to verify failure**

Run: `cd ros2_ws && PYTHONPATH=src/stage_autonomous_nav pytest src/stage_autonomous_nav/test/test_ground_truth_localizer.py -v`

Expected: FAIL because the conversion helpers are undefined.

- [ ] **Step 3: Implement the localizer node**

Implement `GroundTruthLocalizer(Node)` as follows:

```python
class GroundTruthLocalizer(Node):
    def __init__(self):
        super().__init__('ground_truth_localizer')
        self.declare_parameter('ground_truth_topic', '/ground_truth')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 20.0)
        self._ground_truth = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
```

Subscribe to `Odometry` on `ground_truth_topic`. In a timer callback, return without publishing until a ground-truth message exists. Look up `odom_frame` to `base_frame` at `Time()`. Convert both poses to planar values, call `map_to_odom`, and broadcast a transform stamped with the ground-truth timestamp (or the node clock when its stamp is zero). Catch only `TransformException`; log it at throttled warning level and publish nothing for that cycle. `main()` initializes, spins and destroys the node cleanly.

- [ ] **Step 4: Run the unit suite**

Run: `cd ros2_ws && PYTHONPATH=src/stage_autonomous_nav pytest src/stage_autonomous_nav/test/test_ground_truth_localizer.py -v`

Expected: PASS, 4 tests.

- [ ] **Step 5: Build the package**

Run: `cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select stage_autonomous_nav`

Expected: successful build and the `ground_truth_localizer` executable installed under `install/stage_autonomous_nav/lib/stage_autonomous_nav/`.

- [ ] **Step 6: Commit the localizer**

```bash
git add ros2_ws/src/stage_autonomous_nav
git commit -m "feat: publish map to odom from Stage ground truth"
```

### Task 4: Add map, Nav2 parameters and RViz configuration

**Files:**
- Create: `ros2_ws/src/stage_autonomous_nav/maps/cave.pgm`
- Create: `ros2_ws/src/stage_autonomous_nav/maps/cave.yaml`
- Create: `ros2_ws/src/stage_autonomous_nav/config/nav2_cave.yaml`
- Create: `ros2_ws/src/stage_autonomous_nav/rviz/cave_navigation.rviz`
- Create: `ros2_ws/src/stage_autonomous_nav/test/test_navigation_assets.py`

**Interfaces:**
- Consumes: Stage's 16 m × 16 m `cave` bitmap and `/base_scan` LaserScan.
- Produces: a map-server YAML at `maps/cave.yaml`, Nav2 parameters wired to `map`, `odom`, `base_link`, `/odom`, and `/base_scan`, plus a usable RViz config.

- [ ] **Step 1: Write failing static-asset tests**

Create `test/test_navigation_assets.py` to load YAML with `yaml.safe_load` and assert:

```python
assert map_data['resolution'] == 0.02
assert map_data['origin'] == [-8.0, -8.0, 0.0]
assert nav_data['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'map'
assert nav_data['local_costmap']['local_costmap']['ros__parameters']['observation_sources'] == 'scan'
assert nav_data['local_costmap']['local_costmap']['ros__parameters']['scan']['topic'] == '/base_scan'
```

- [ ] **Step 2: Run tests to verify failure**

Run: `cd ros2_ws && PYTHONPATH=src/stage_autonomous_nav pytest src/stage_autonomous_nav/test/test_navigation_assets.py -v`

Expected: FAIL because the resource files do not exist.

- [ ] **Step 3: Add an occupancy map consistent with Stage `cave`**

After importing `stage_ros2`, generate `cave.pgm` from `src/stage_ros2/world/bitmaps/cave.png`. Run this from `ros2_ws`:

```bash
python3 - <<'PY'
from pathlib import Path
from PIL import Image

source = Path('src/stage_ros2/world/bitmaps/cave.png')
target = Path('src/stage_autonomous_nav/maps/cave.pgm')
image = Image.open(source).convert('L').resize((800, 800), Image.Resampling.NEAREST)
image.point(lambda value: 0 if value < 128 else 254).save(target)
PY
```

The 800 × 800 PGM at 0.02 m/pixel preserves the `[-8, -8]` to `[8, 8]` bounds. Do not draw the dynamic green block at `(5, 4)` into this PGM. Create `cave.yaml`:

```yaml
image: cave.pgm
mode: trinary
resolution: 0.02
origin: [-8.0, -8.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

Record the source asset and its upstream license in a comment at the top of `cave.yaml`.

- [ ] **Step 4: Add a minimal complete Nav2 parameter file**

Base `nav2_cave.yaml` on the Jazzy `nav2_bringup` parameter file and retain every parameter required by `navigation_launch.py`. Set `use_sim_time: true`, `robot_base_frame: base_link`, `odom_frame: odom`, `global_frame: map`, and `odom_topic: /odom`. Configure both costmaps with `robot_radius: 0.25`, inflation layer `inflation_radius: 0.45`, and a `scan` obstacle layer whose topic is `/base_scan`, type is `LaserScan`, `marking: true`, `clearing: true`, and `max_obstacle_height: 2.0`. Configure `velocity_smoother` to accept `cmd_vel_nav`, publish `cmd_vel`, use `/odom`, and cap linear x at `0.4 m/s` and angular z at `1.0 rad/s`.

- [ ] **Step 5: Add RViz defaults**

Create `cave_navigation.rviz` with fixed frame `map`; displays for Map `/map`, LaserScan `/base_scan`, Path `/plan`, Path `/local_plan`, TF, global/local costmaps, and the `rviz_default_plugins/SetGoal` tool. Set the initial tool to `SetGoal` so RViz publishes the normal Nav2 goal topic.

- [ ] **Step 6: Run static tests**

Run: `cd ros2_ws && PYTHONPATH=src/stage_autonomous_nav pytest src/stage_autonomous_nav/test/test_navigation_assets.py -v`

Expected: PASS.

- [ ] **Step 7: Commit navigation assets**

```bash
git add ros2_ws/src/stage_autonomous_nav
git commit -m "feat: add cave map and Nav2 configuration"
```

### Task 5: Integrate Stage, map server, Nav2 and RViz in one launch

**Files:**
- Create: `ros2_ws/src/stage_autonomous_nav/launch/cave_navigation.launch.py`
- Create: `ros2_ws/src/stage_autonomous_nav/stage_autonomous_nav/launch/__init__.py`
- Create: `ros2_ws/src/stage_autonomous_nav/stage_autonomous_nav/launch/cave_navigation.py`
- Create: `ros2_ws/src/stage_autonomous_nav/test/test_launch_description.py`

**Interfaces:**
- Consumes: `stage_ros2/launch/stage.launch.py`, `nav2_bringup/launch/navigation_launch.py`, package resources from Task 4 and `ground_truth_localizer` from Task 3.
- Produces: `ros2 launch stage_autonomous_nav cave_navigation.launch.py`.
- Arguments: `use_sim_time:=true`, `world:=cave`, `map_frame:=map`, `odom_frame:=odom`, `base_frame:=base_link`, `scan_topic:=/base_scan`, `ground_truth_topic:=/ground_truth`, `cmd_vel_topic:=/cmd_vel`, `rviz:=true`.

- [ ] **Step 1: Write a failing launch-description test**

Create `test/test_launch_description.py`:

```python
from stage_autonomous_nav.launch.cave_navigation import generate_launch_description


def test_cave_navigation_declares_supported_arguments():
    names = {
        action.name
        for action in generate_launch_description().entities
        if hasattr(action, 'name')
    }
    assert {'world', 'map_frame', 'odom_frame', 'base_frame', 'scan_topic', 'rviz'} <= names
```

Name the importable launch module `cave_navigation.py` in the Python package and make `launch/cave_navigation.launch.py` a thin installed wrapper importing its `generate_launch_description`; this keeps the launch description unit-testable.

- [ ] **Step 2: Run test to verify failure**

Run: `cd ros2_ws && PYTHONPATH=src/stage_autonomous_nav pytest src/stage_autonomous_nav/test/test_launch_description.py -v`

Expected: FAIL because the launch module is undefined.

- [ ] **Step 3: Implement the launch description**

The launch must:

1. Declare all arguments listed above.
2. Include `stage_ros2`'s `stage.launch.py` with `world`, `enforce_prefixes: false`, `one_tf_tree: true`, `use_stamped_velocity: false` and `use_ackermann: false`.
3. Start `ground_truth_localizer` with its topic/frame parameters and `use_sim_time`.
4. Start `nav2_map_server/map_server` with `yaml_filename` pointing at `maps/cave.yaml` and `use_sim_time`.
5. Start a `nav2_lifecycle_manager` named `lifecycle_manager_map` with `autostart: true` and `node_names: ['map_server']`.
6. Include `nav2_bringup/launch/navigation_launch.py` with `params_file` set to `config/nav2_cave.yaml`, `use_sim_time`, `autostart: true`, and composition disabled.
7. Start RViz conditionally with `rviz/cave_navigation.rviz` and `use_sim_time`.

Configure Nav2's `velocity_smoother` output as `/cmd_vel`, matching the default Stage subscriber; a future multi-robot extension can add velocity-topic remapping.

- [ ] **Step 4: Run the launch test**

Run: `cd ros2_ws && PYTHONPATH=src/stage_autonomous_nav pytest src/stage_autonomous_nav/test/test_launch_description.py -v`

Expected: PASS.

- [ ] **Step 5: Install and inspect launch metadata**

Run: `cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select stage_autonomous_nav && source install/setup.bash && ros2 launch stage_autonomous_nav cave_navigation.launch.py --show-args`

Expected: the eight documented launch arguments, with `world` defaulting to `cave`.

- [ ] **Step 6: Commit the integrated launch**

```bash
git add ros2_ws/src/stage_autonomous_nav
git commit -m "feat: launch autonomous navigation in Stage cave"
```

### Task 6: Document operation and perform end-to-end verification

**Files:**
- Create: `ros2_ws/src/stage_autonomous_nav/README.md`
- Modify: `ros2_ws/src/stage_autonomous_nav/package.xml`

**Interfaces:**
- Consumes: a built workspace and the single integrated launch.
- Produces: reproducible user instructions and a documented acceptance procedure.

- [ ] **Step 1: Write the package README**

Document prerequisites, importing `stage_nav.repos`, `rosdep install`, `colcon build`, sourcing the workspace, and this launch command:

```bash
ros2 launch stage_autonomous_nav cave_navigation.launch.py
```

Explain that the Stage GUI and RViz are visible via the existing VNC/noVNC setup. Include the RViz workflow: select **2D Goal Pose**, click-drag a free location in the cave, inspect `/plan` and `/base_scan`, then wait for the Nav2 goal result. Document `/ground_truth` as simulation-only and state that it must be replaced by AMCL or another localization source for real hardware.

- [ ] **Step 2: Add test metadata**

Add `test_depend` entries for `ament_pytest`, `python3-pytest` and `python3-yaml`. Add the `pytest.ini` configuration in `setup.cfg` only if the test runner needs `python_files = test_*.py`; do not add unrelated lint tools.

- [ ] **Step 3: Run all automated verification**

Run:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select stage_autonomous_nav
source install/setup.bash
colcon test --packages-select stage_autonomous_nav
colcon test-result --verbose
```

Expected: build exits 0; all package tests pass; `test-result` reports no failures.

- [ ] **Step 4: Run the manual acceptance scenario**

Start the integrated launch in the container with `DISPLAY=:1`. In RViz, issue a free-space goal such as `(-1.0, -5.5)` with a valid orientation. Verify all of the following before declaring success:

```bash
ros2 topic echo /tf --once
ros2 topic echo /base_scan --once
ros2 topic echo /cmd_vel --once
ros2 lifecycle get /map_server
ros2 lifecycle get /controller_server
```

Expected: `map -> odom` exists in TF, `/base_scan` has ranges, Nav2 emits at least one nonzero velocity during travel, and both lifecycle nodes are `active`. Then send a goal whose route passes near `(5, 4)` and visually confirm that the robot does not collide with the block.

- [ ] **Step 5: Commit documentation and verification changes**

```bash
git add ros2_ws/src/stage_autonomous_nav
git commit -m "docs: explain Stage autonomous navigation workflow"
```
