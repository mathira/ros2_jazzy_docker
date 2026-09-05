"""ROS safety wrapper and sole ``/cmd_vel`` publisher for exploration actions."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

from turtleboot3_autonomous_nav.control import (
    RECOVER,
    ControlConfig,
    TwistDecision,
    control_sector_ranges,
    safe_twist,
    stale_twist,
)


class SafeMotionController(Node):
    """Turn policy actions into fail-safe velocity commands at a fixed rate."""

    def __init__(self) -> None:
        super().__init__('safe_motion_controller')
        self.declare_parameter('stop_distance', 0.20)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('soft_turn_speed', 0.5)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('emergency_turn_speed', 1.0)
        self.declare_parameter('recovery_turn_speed', 1.2)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('sensor_timeout', 0.5)
        self.declare_parameter('action_timeout', 1.0)
        self.declare_parameter('progress_distance', 0.03)
        self.declare_parameter('progress_timeout', 3.0)
        self.declare_parameter('coverage_delta', 0.0001)
        self.declare_parameter('control_rate', 10.0)

        self._config = ControlConfig(
            stop_distance=float(self.get_parameter('stop_distance').value),
            linear_speed=float(self.get_parameter('linear_speed').value),
            soft_turn_speed=float(self.get_parameter('soft_turn_speed').value),
            turn_speed=float(self.get_parameter('turn_speed').value),
            emergency_turn_speed=float(
                self.get_parameter('emergency_turn_speed').value
            ),
            recovery_turn_speed=float(
                self.get_parameter('recovery_turn_speed').value
            ),
            max_linear_speed=float(self.get_parameter('max_linear_speed').value),
            max_angular_speed=float(self.get_parameter('max_angular_speed').value),
        )
        self._sensor_timeout_ns = int(
            float(self.get_parameter('sensor_timeout').value) * 1_000_000_000
        )
        self._action_timeout_ns = int(
            float(self.get_parameter('action_timeout').value) * 1_000_000_000
        )
        self._progress_distance = float(self.get_parameter('progress_distance').value)
        self._progress_timeout_ns = int(
            float(self.get_parameter('progress_timeout').value) * 1_000_000_000
        )
        self._coverage_delta = float(self.get_parameter('coverage_delta').value)
        now = self._now_ns()
        self._scan: LaserScan | None = None
        self._scan_time_ns: int | None = None
        self._action = RECOVER
        self._action_time_ns: int | None = None
        self._last_position: tuple[float, float] | None = None
        self._last_coverage: float | None = None
        self._last_progress_ns = now

        self._cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._intervention_publisher = self.create_publisher(
            Bool, '/safety_intervention', 10
        )
        self._recovery_publisher = self.create_publisher(Bool, '/recovery_active', 10)
        self.create_subscription(Int32, '/exploration_action', self._on_action, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odometry, 10)
        self.create_subscription(Float32, '/coverage_metrics', self._on_coverage, 10)
        control_rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / max(control_rate, 1.0), self._on_control_timer)

    def _on_action(self, message: Int32) -> None:
        self._action = int(message.data)
        self._action_time_ns = self._now_ns()

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = message
        self._scan_time_ns = self._now_ns()

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self._last_position is None:
            self._last_position = current
            return
        if math.dist(current, self._last_position) >= self._progress_distance:
            self._last_progress_ns = self._now_ns()
            self._last_position = current

    def _on_coverage(self, message: Float32) -> None:
        coverage = float(message.data)
        if self._last_coverage is None:
            self._last_coverage = coverage
            return
        if coverage - self._last_coverage >= self._coverage_delta:
            self._last_progress_ns = self._now_ns()
        self._last_coverage = coverage

    def _on_control_timer(self) -> None:
        now = self._now_ns()
        if self._data_is_stale(now):
            self._publish_decision(stale_twist())
            return
        assert self._scan is not None
        decision = safe_twist(
            self._action,
            _control_sector_ranges(self._scan),
            now - self._last_progress_ns >= self._progress_timeout_ns,
            self._config,
        )
        self._publish_decision(decision)

    def _data_is_stale(self, now_ns: int) -> bool:
        return (
            self._scan is None
            or self._scan_time_ns is None
            or self._action_time_ns is None
            or now_ns - self._scan_time_ns > self._sensor_timeout_ns
            or now_ns - self._action_time_ns > self._action_timeout_ns
        )

    def _publish_decision(self, decision: TwistDecision) -> None:
        command = Twist()
        command.linear.x = decision.linear_x
        command.angular.z = decision.angular_z
        self._cmd_publisher.publish(command)
        self._intervention_publisher.publish(Bool(data=decision.intervention))
        self._recovery_publisher.publish(Bool(data=decision.recovery))

    def publish_stop(self) -> None:
        """Issue a final zero command before node shutdown."""
        self._publish_decision(stale_twist())

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds


def _control_sector_ranges(message: LaserScan) -> np.ndarray:
    """Return minimum ``[front, left, right]`` clearances from a scan."""
    return control_sector_ranges(
        np.asarray(message.ranges, dtype=float),
        message.angle_min,
        message.angle_increment,
        message.range_max,
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafeMotionController()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
