import os
import signal
import subprocess
import sys
import time
import uuid

import pytest


rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402


def _is_zero(message):
    return message.linear.x == 0.0 and message.angular.z == 0.0


def test_sigint_publishes_final_stop_before_ros_context_shutdown():
    suffix = uuid.uuid4().hex
    command_topic = f"/pid_lifecycle_{suffix}/cmd_vel"
    odom_topic = f"/pid_lifecycle_{suffix}/odom"
    code = "from stage_pid_navigation.pid_navigator import main; main()"
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            code,
            "--ros-args",
            "-p",
            "require_scan:=false",
            "-p",
            "goal_x:=1.0",
            "-p",
            f"odom_topic:={odom_topic}",
            "-p",
            f"cmd_vel_topic:={command_topic}",
        ],
        env={**os.environ, "PYTHONUNBUFFERED": "1"},
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    rclpy.init()
    observer = Node(f"pid_lifecycle_observer_{suffix}")
    odom_publisher = observer.create_publisher(Odometry, odom_topic, 10)
    messages = []
    observer.create_subscription(Twist, command_topic, messages.append, 10)
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0

    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not any(
            not _is_zero(message) for message in messages
        ):
            odom_publisher.publish(odometry)
            rclpy.spin_once(observer, timeout_sec=0.05)

        assert any(not _is_zero(message) for message in messages)
        process.send_signal(signal.SIGINT)

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(observer, timeout_sec=0.05)
        for _ in range(10):
            rclpy.spin_once(observer, timeout_sec=0.02)

        output = process.communicate(timeout=1.0)[0]
        assert process.returncode == 0, output
        assert _is_zero(messages[-1]), output
        assert "RCLError" not in output
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5.0)
        observer.destroy_node()
        rclpy.shutdown()
