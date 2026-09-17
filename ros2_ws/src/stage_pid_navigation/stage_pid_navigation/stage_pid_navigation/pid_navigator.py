"""ROS 2 adapter for the pure Stage PID navigation controller."""

import math
import signal
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from stage_pid_navigation.control import (
    Command,
    NavigationConfig,
    PidController,
    Pose2D,
    ScanSummary,
    WallFollower,
    compute_command,
    summarize_scan,
)
from stage_pid_navigation.global_planner import cave_planner


PARAMETER_DEFAULTS = {
    "goal_x": 5.0,
    "goal_y": 4.0,
    "odom_topic": "/ground_truth",
    "scan_topic": "/base_scan",
    "cmd_vel_topic": "/cmd_vel",
    "control_rate": 10.0,
    "kp": 1.8,
    "ki": 0.0,
    "kd": 0.15,
    "integral_limit": 1.0,
    "max_linear_speed": 0.35,
    "max_angular_speed": 1.2,
    "heading_stop_threshold": 0.7,
    "goal_tolerance": 0.15,
    "slowdown_distance": 0.9,
    "stop_distance": 0.35,
    "front_sector_angle": 0.7,
    "require_scan": True,
    "goal_standoff": 0.65,
    "wall_distance": 0.55,
    "wall_follow_linear_speed": 0.18,
    "wall_kp": 1.5,
    "stuck_timeout": 8.0,
    "recovery_turn_duration": 1.2,
    "use_global_planner": True,
}


class _ShutdownRequested(Exception):
    pass


def _request_shutdown(_signum, _frame) -> None:
    raise _ShutdownRequested


class PidNavigator(Node):
    """Connect odometry and laser scans to velocity commands."""

    def __init__(self):
        super().__init__("pid_navigator")
        parameters = {
            name: self.declare_parameter(name, default).value
            for name, default in PARAMETER_DEFAULTS.items()
        }

        self._goal = (float(parameters["goal_x"]), float(parameters["goal_y"]))
        self._require_scan = bool(parameters["require_scan"])
        self._goal_standoff = float(parameters["goal_standoff"])
        self._control_period = 1.0 / float(parameters["control_rate"])
        self._config = NavigationConfig(
            max_linear_speed=float(parameters["max_linear_speed"]),
            max_angular_speed=float(parameters["max_angular_speed"]),
            heading_stop_threshold=float(parameters["heading_stop_threshold"]),
            goal_tolerance=float(parameters["goal_tolerance"]),
            slowdown_distance=float(parameters["slowdown_distance"]),
            stop_distance=float(parameters["stop_distance"]),
            front_sector_angle=float(parameters["front_sector_angle"]),
        )
        self._pid = PidController(
            kp=float(parameters["kp"]),
            ki=float(parameters["ki"]),
            kd=float(parameters["kd"]),
            integral_limit=float(parameters["integral_limit"]),
        )
        self._navigator = WallFollower(
            self._config, self._pid,
            goal_standoff=float(parameters["goal_standoff"]),
            wall_distance=float(parameters["wall_distance"]),
            wall_follow_linear_speed=float(parameters["wall_follow_linear_speed"]),
            wall_kp=float(parameters["wall_kp"]),
            stuck_timeout=float(parameters["stuck_timeout"]),
            recovery_turn_duration=float(parameters["recovery_turn_duration"]),
        )
        if bool(parameters["use_global_planner"]):
            self._planner = cave_planner()
            self._navigation_goal = self._goal
            if not self._planner.is_free(self._goal):
                self.get_logger().warning(
                    "El objetivo solicitado está dentro de un obstáculo. "
                    "Buscaré el punto alcanzable más cercano."
                )
        else:
            self._navigation_goal = self._goal
        self._waypoints = []
        self._pose: Optional[Pose2D] = None
        self._scan: Optional[ScanSummary] = None
        self._reached_goal = False
        self._planner_error_reported = False
        self._last_control_time = time.monotonic()

        self.get_logger().info(
            f"Navegación iniciada: objetivo={self._goal}, "
            f"objetivo seguro={self._navigation_goal}"
        )

        self._cmd_vel_publisher = self.create_publisher(
            Twist, str(parameters["cmd_vel_topic"]), 10
        )
        self._odom_subscription = self.create_subscription(
            Odometry, str(parameters["odom_topic"]), self._on_odom, 10
        )
        self._scan_subscription = self.create_subscription(
            LaserScan, str(parameters["scan_topic"]), self._on_scan, 10
        )
        self._control_timer = self.create_timer(
            self._control_period, self._on_control_timer
        )

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        self._pose = Pose2D(
            x=position.x,
            y=position.y,
            yaw=math.atan2(sin_yaw, cos_yaw),
        )

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = summarize_scan(
            ranges=message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            front_sector_angle=self._config.front_sector_angle,
        )

    def _on_control_timer(self) -> None:
        now = time.monotonic()
        dt = now - self._last_control_time
        self._last_control_time = now

        if self._reached_goal:
            self._publish_stop()
            return

        if self._pose is None or (self._require_scan and self._scan is None):
            self._publish_stop()
            return

        # Keeps the adapter easy to exercise with lightweight test doubles.
        navigation_goal = getattr(self, "_navigation_goal", self._goal)
        scan = self._scan if self._scan is not None else ScanSummary.clear()
        goal_distance = math.dist((self._pose.x, self._pose.y), navigation_goal)
        arrival_distance = getattr(self, "_goal_standoff", 0.0) + self._config.goal_tolerance
        if goal_distance <= arrival_distance:
            command = Command(0.0, 0.0, reached_goal=True)
        else:
            if not hasattr(self, "_planner"):
                command = compute_command(
                    pose=self._pose, goal=navigation_goal, scan=scan, config=self._config,
                    pid=self._pid, dt=dt
                )
            else:
                if not self._waypoints:
                    try:
                        planning_goal = (
                            navigation_goal[0] - (navigation_goal[0] - self._pose.x) / goal_distance * self._goal_standoff,
                            navigation_goal[1] - (navigation_goal[1] - self._pose.y) / goal_distance * self._goal_standoff,
                        )
                        self._waypoints = self._planner.plan((self._pose.x, self._pose.y), planning_goal)[1:]
                    except ValueError as error:
                        try:
                            self._waypoints = self._planner.plan_to_closest_reachable(
                                (self._pose.x, self._pose.y), planning_goal
                            )[1:]
                            if self._waypoints:
                                self._navigation_goal = self._waypoints[-1]
                                self.get_logger().warning(
                                    "La ruta directa no es posible; "
                                    f"uso un objetivo alcanzable en {self._navigation_goal}."
                                )
                        except ValueError as fallback_error:
                            if not self._planner_error_reported:
                                self.get_logger().error(
                                    f"No se pudo calcular la ruta: {error}. {fallback_error}"
                                )
                                self._planner_error_reported = True
                            self._publish_stop()
                            return
                while len(self._waypoints) > 1 and math.dist((self._pose.x, self._pose.y), self._waypoints[0]) <= self._config.goal_tolerance:
                    self._waypoints.pop(0)
                command = compute_command(
                    pose=self._pose, goal=self._waypoints[0], scan=scan, config=self._config,
                    pid=self._pid, dt=dt
                )
        if command.reached_goal:
            self._reached_goal = True
            self.get_logger().info("Goal reached")
            self._publish_stop()
            return

        twist = Twist()
        twist.linear.x = command.linear
        twist.angular.z = command.angular
        self._cmd_vel_publisher.publish(twist)

    def _publish_stop(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def stop(self) -> None:
        """Publish a final stop while the ROS context is still valid."""
        self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    navigator = PidNavigator()
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, _request_shutdown)
    try:
        rclpy.spin(navigator)
    except (KeyboardInterrupt, _ShutdownRequested):
        pass
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
        if rclpy.ok():
            navigator.stop()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
