import importlib
import math
import sys
from types import ModuleType, SimpleNamespace

import pytest


class _Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _Odometry:
    def __init__(self):
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


class _LaserScan:
    def __init__(self):
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Context:
    def __init__(self):
        self.shutdown_callbacks = []

    def on_shutdown(self, callback):
        self.shutdown_callbacks.append(callback)


class _Logger:
    def __init__(self):
        self.info_messages = []

    def info(self, message):
        self.info_messages.append(message)


class _Node:
    def __init__(self, name):
        self.node_name = name
        self.context = _Context()
        self.declared_parameters = {}
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.logger = _Logger()

    def declare_parameter(self, name, default_value):
        self.declared_parameters[name] = default_value
        return SimpleNamespace(value=default_value)

    def create_publisher(self, message_type, topic, qos_depth):
        publisher = _Publisher(topic)
        self.publishers.append((message_type, publisher, qos_depth))
        return publisher

    def create_subscription(self, message_type, topic, callback, qos_depth):
        subscription = (message_type, topic, callback, qos_depth)
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        timer = (period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        pass


def _load_adapter_with_fake_ros():
    module_names = (
        "rclpy",
        "rclpy.node",
        "rclpy.signals",
        "geometry_msgs",
        "geometry_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
    )
    saved_modules = {name: sys.modules.get(name) for name in module_names}

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda: None
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = _Node
    rclpy.node = rclpy_node
    rclpy_signals = ModuleType("rclpy.signals")
    rclpy_signals.SignalHandlerOptions = SimpleNamespace(NO="NO")
    rclpy.signals = rclpy_signals

    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = _Twist
    geometry_msgs.msg = geometry_msgs_msg

    nav_msgs = ModuleType("nav_msgs")
    nav_msgs_msg = ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = _Odometry
    nav_msgs.msg = nav_msgs_msg

    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs_msg = ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = _LaserScan
    sensor_msgs.msg = sensor_msgs_msg

    replacements = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.signals": rclpy_signals,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    }
    sys.modules.update(replacements)
    try:
        return importlib.import_module("stage_pid_navigation.pid_navigator")
    finally:
        for name, original in saved_modules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


adapter = _load_adapter_with_fake_ros()

from stage_pid_navigation.control import (  # noqa: E402
    Command,
    NavigationConfig,
    PidController,
    Pose2D,
    ScanSummary,
)


def make_navigator_for_test(*, require_scan=True):
    navigator = adapter.PidNavigator.__new__(adapter.PidNavigator)
    navigator._pose = None
    navigator._scan = None
    navigator._require_scan = require_scan
    navigator._goal = (1.0, 0.0)
    navigator._config = NavigationConfig()
    navigator._pid = PidController(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0)
    navigator._control_period = 0.1
    navigator._last_control_time = 10.0
    navigator._reached_goal = False
    navigator._cmd_vel_publisher = _Publisher("/cmd_vel")
    navigator.get_logger = lambda: _Logger()
    return navigator


def odometry_with_yaw(yaw):
    message = _Odometry()
    message.pose.pose.position.x = 2.0
    message.pose.pose.position.y = -1.0
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def last_published_twist(navigator):
    return navigator._cmd_vel_publisher.messages[-1]


def assert_zero_twist(message):
    assert message.linear.x == 0.0
    assert message.linear.y == 0.0
    assert message.linear.z == 0.0
    assert message.angular.x == 0.0
    assert message.angular.y == 0.0
    assert message.angular.z == 0.0


def test_constructor_declares_parameters_and_wires_ros_interfaces():
    navigator = adapter.PidNavigator()

    assert navigator.declared_parameters == {
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
    assert navigator._cmd_vel_publisher.topic == "/cmd_vel"
    assert {subscription[1] for subscription in navigator.subscriptions} == {
        "/ground_truth",
        "/base_scan",
    }
    assert navigator.timers[0][0] == pytest.approx(0.1)
    assert navigator.context.shutdown_callbacks == []


def test_odometry_callback_converts_quaternion_to_planar_pose():
    navigator = make_navigator_for_test()

    navigator._on_odom(odometry_with_yaw(math.pi / 2.0))

    assert navigator._pose.x == 2.0
    assert navigator._pose.y == -1.0
    assert navigator._pose.yaw == pytest.approx(math.pi / 2.0)


def test_scan_callback_summarizes_ranges_for_the_controller():
    navigator = make_navigator_for_test()
    scan = _LaserScan()
    scan.ranges = [0.3, 0.8, 0.4]
    scan.angle_min = -0.5
    scan.angle_increment = 0.5

    navigator._on_scan(scan)

    assert navigator._scan == ScanSummary(front=0.8, left=0.4, right=0.3)


@pytest.mark.parametrize(
    ("pose", "require_scan"),
    [
        (None, False),
        (Pose2D(0.0, 0.0, 0.0), True),
    ],
)
def test_timer_publishes_stop_without_pose_or_required_scan(pose, require_scan):
    navigator = make_navigator_for_test(require_scan=require_scan)
    navigator._pose = pose

    navigator._on_control_timer()

    assert_zero_twist(last_published_twist(navigator))


def test_timer_converts_command_to_twist(monkeypatch):
    navigator = make_navigator_for_test(require_scan=False)
    navigator._pose = Pose2D(0.0, 0.0, 0.0)
    monkeypatch.setattr(
        adapter,
        "compute_command",
        lambda *args, **kwargs: Command(0.2, -0.4, False, False),
    )

    navigator._on_control_timer()

    assert last_published_twist(navigator).linear.x == 0.2
    assert last_published_twist(navigator).angular.z == -0.4


def test_timer_publishes_stop_after_goal_is_reached(monkeypatch):
    navigator = make_navigator_for_test(require_scan=False)
    navigator._pose = Pose2D(0.0, 0.0, 0.0)
    monkeypatch.setattr(
        adapter,
        "compute_command",
        lambda *args, **kwargs: Command(0.2, -0.4, True, False),
    )

    navigator._on_control_timer()

    assert_zero_twist(last_published_twist(navigator))


def test_goal_arrival_is_latched_and_logged_once(monkeypatch):
    navigator = make_navigator_for_test(require_scan=False)
    navigator._pose = Pose2D(0.0, 0.0, 0.0)
    logger = _Logger()
    navigator.get_logger = lambda: logger
    calls = []

    def reached_goal(*args, **kwargs):
        calls.append((args, kwargs))
        return Command(0.0, 0.0, True, False)

    monkeypatch.setattr(adapter, "compute_command", reached_goal)

    navigator._on_control_timer()
    navigator._pose = Pose2D(10.0, 10.0, 0.0)
    navigator._on_control_timer()

    assert len(calls) == 1
    assert len(navigator._cmd_vel_publisher.messages) == 2
    assert_zero_twist(navigator._cmd_vel_publisher.messages[0])
    assert_zero_twist(navigator._cmd_vel_publisher.messages[1])
    assert logger.info_messages == ["Goal reached"]


def test_timer_passes_elapsed_monotonic_time_to_controller(monkeypatch):
    navigator = make_navigator_for_test(require_scan=False)
    navigator._pose = Pose2D(0.0, 0.0, 0.0)
    captured = {}
    monkeypatch.setattr(adapter.time, "monotonic", lambda: 10.35)

    def capture_command(*args, **kwargs):
        captured.update(kwargs)
        return Command(0.2, 0.0, False, False)

    monkeypatch.setattr(adapter, "compute_command", capture_command)

    navigator._on_control_timer()

    assert captured["dt"] == pytest.approx(0.35)


def test_main_publishes_stop_before_normal_shutdown(monkeypatch):
    events = []

    class _Navigator:
        def stop(self):
            events.append("stop")

        def destroy_node(self):
            events.append("destroy")

    monkeypatch.setattr(adapter, "PidNavigator", _Navigator)
    monkeypatch.setattr(
        adapter.rclpy,
        "init",
        lambda **kwargs: events.append(("init", kwargs)),
    )
    monkeypatch.setattr(adapter.rclpy, "spin", lambda node: events.append("spin"))
    monkeypatch.setattr(adapter.rclpy, "ok", lambda: True, raising=False)
    monkeypatch.setattr(adapter.rclpy, "shutdown", lambda: events.append("shutdown"))

    adapter.main(args=["--ros-args"])

    assert events == [
        ("init", {"args": ["--ros-args"], "signal_handler_options": "NO"}),
        "spin",
        "stop",
        "destroy",
        "shutdown",
    ]


def test_main_skips_ros_operations_after_external_context_shutdown(monkeypatch):
    events = []
    context = {"valid": True}

    class _Navigator:
        def stop(self):
            if not context["valid"]:
                raise RuntimeError("publisher context is invalid")
            events.append("stop")

        def destroy_node(self):
            events.append("destroy")

    def spin(_node):
        events.append("spin")
        context["valid"] = False

    def shutdown():
        if not context["valid"]:
            raise RuntimeError("context is already shut down")
        events.append("shutdown")

    monkeypatch.setattr(adapter, "PidNavigator", _Navigator)
    monkeypatch.setattr(
        adapter.rclpy,
        "init",
        lambda **kwargs: events.append(("init", kwargs)),
    )
    monkeypatch.setattr(adapter.rclpy, "spin", spin)
    monkeypatch.setattr(
        adapter.rclpy, "ok", lambda: context["valid"], raising=False
    )
    monkeypatch.setattr(adapter.rclpy, "shutdown", shutdown)

    adapter.main(args=["--ros-args"])

    assert events == [
        ("init", {"args": ["--ros-args"], "signal_handler_options": "NO"}),
        "spin",
        "destroy",
    ]
