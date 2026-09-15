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


class _Node:
    def __init__(self, name):
        self.node_name = name
        self.context = _Context()
        self.declared_parameters = {}
        self.publishers = []
        self.subscriptions = []
        self.timers = []

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

    def destroy_node(self):
        pass


def _load_adapter_with_fake_ros():
    module_names = (
        "rclpy",
        "rclpy.node",
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
    navigator._cmd_vel_publisher = _Publisher("/cmd_vel")
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

    assert set(navigator.declared_parameters) == {
        "goal_x",
        "goal_y",
        "odom_topic",
        "scan_topic",
        "cmd_vel_topic",
        "control_rate",
        "kp",
        "ki",
        "kd",
        "integral_limit",
        "max_linear_speed",
        "max_angular_speed",
        "heading_stop_threshold",
        "goal_tolerance",
        "slowdown_distance",
        "stop_distance",
        "front_sector_angle",
        "require_scan",
    }
    assert navigator._cmd_vel_publisher.topic == "/cmd_vel"
    assert {subscription[1] for subscription in navigator.subscriptions} == {
        "/odom",
        "/base_scan",
    }
    assert navigator.timers[0][0] == pytest.approx(0.1)
    assert navigator.context.shutdown_callbacks == [navigator._on_shutdown]


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


def test_shutdown_callback_publishes_stop():
    navigator = make_navigator_for_test()

    navigator._on_shutdown()

    assert_zero_twist(last_published_twist(navigator))
