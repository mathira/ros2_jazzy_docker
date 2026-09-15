# Stage PID Navigation

`stage_pid_navigation` drives a differential robot in Stage toward a planar
goal. The ROS node reads odometry and LiDAR data, uses an angular PID
controller, slows or turns away from nearby obstacles, and publishes
`geometry_msgs/msg/Twist` commands.

## Build

From the repository root:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stage_pid_navigation
source install/setup.bash
```

Source both setup files in every new terminal used below:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Single-robot operation

Start the unprefixed single-robot Stage world in one sourced terminal:

```bash
ros2 launch stage_ros2 stage.launch.py world:=cave
```

In another sourced terminal, start navigation with a goal expressed in the
odometry frame:

```bash
ros2 launch stage_pid_navigation pid_navigation.launch.py \
  goal_x:=2.0 goal_y:=1.5
```

The default topics are `/odom`, `/base_scan`, and `/cmd_vel`. The node remains
stopped until it has received odometry and, by default, a laser scan. It also
publishes a zero velocity after reaching the goal and during shutdown.

## Multi-robot topics

For `robot_0` in a prefixed Stage world, select that robot's interfaces:

```bash
ros2 launch stage_pid_navigation pid_navigation.launch.py \
  goal_x:=2.0 goal_y:=1.5 \
  odom_topic:=/robot_0/odom \
  scan_topic:=/robot_0/base_scan \
  cmd_vel_topic:=/robot_0/cmd_vel
```

Run a separate navigator with distinct topic arguments for each additional
robot.

## Parameters

All parameters are also launch arguments.

| Parameter | Default | Description |
| --- | ---: | --- |
| `goal_x` | `0.0` | Goal x-coordinate in metres in the odometry frame. |
| `goal_y` | `0.0` | Goal y-coordinate in metres in the odometry frame. |
| `odom_topic` | `/odom` | `nav_msgs/msg/Odometry` input topic. |
| `scan_topic` | `/base_scan` | `sensor_msgs/msg/LaserScan` input topic. |
| `cmd_vel_topic` | `/cmd_vel` | `geometry_msgs/msg/Twist` output topic. |
| `control_rate` | `10.0` | Fixed control-loop frequency in hertz. |
| `kp` | `1.0` | Proportional gain for heading error. |
| `ki` | `0.0` | Integral gain for heading error. |
| `kd` | `0.0` | Derivative gain for heading error. |
| `integral_limit` | `1.0` | Absolute anti-windup limit on accumulated heading error. |
| `max_linear_speed` | `0.3` | Maximum forward speed in metres per second. |
| `max_angular_speed` | `1.0` | Maximum turn rate in radians per second. |
| `heading_stop_threshold` | `0.35` | Absolute heading error in radians above which forward motion stops. |
| `goal_tolerance` | `0.15` | Distance in metres at which the goal is considered reached. |
| `slowdown_distance` | `0.75` | Front-obstacle distance in metres below which forward speed is reduced. |
| `stop_distance` | `0.25` | Front-obstacle distance in metres below which forward motion stops and escape turning begins. |
| `front_sector_angle` | `0.5` | Total angular width in radians of the forward LiDAR sector. |
| `require_scan` | `true` | Keep the robot stopped until a scan arrives; set to `false` only when operating without LiDAR protection. |

## Limitation

Obstacle handling is a local LiDAR reaction, not global path planning. It can
slow, stop, and turn away from nearby objects, but it cannot guarantee reaching
a goal behind a non-traversable obstacle. Use a global planner when the route
requires reasoning around walls, dead ends, or other large obstacles.
