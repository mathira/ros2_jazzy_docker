"""ROS 2 adapter that turns current odometry and LiDAR into a coverage map."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Empty

from turtleboot3_autonomous_nav.grid_mapping import OccupancyGridModel


class CoverageMapper(Node):
    """Publish an odometric occupancy map only after scan and odometry arrive."""

    def __init__(self) -> None:
        super().__init__("coverage_mapper")
        self.declare_parameter("map_width", 400)
        self.declare_parameter("map_height", 400)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("map_origin_x", -10.0)
        self.declare_parameter("map_origin_y", -10.0)
        self.declare_parameter("occupied_threshold", 1)

        self._grid = OccupancyGridModel(
            self.get_parameter("map_width").value,
            self.get_parameter("map_height").value,
            self.get_parameter("map_resolution").value,
            (
                self.get_parameter("map_origin_x").value,
                self.get_parameter("map_origin_y").value,
            ),
            self.get_parameter("occupied_threshold").value,
        )
        self._latest_pose: tuple[float, float, float] | None = None
        self._map_publisher = self.create_publisher(OccupancyGrid, "/coverage_map", 10)
        self._metrics_publisher = self.create_publisher(
            Float32, "/coverage_metrics", 10
        )
        self.create_subscription(Odometry, "/odom", self._on_odometry, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_service(Empty, "/coverage_mapper/reset", self._on_reset)

    def _on_reset(self, _: Empty.Request, response: Empty.Response) -> Empty.Response:
        """Clear the episode map and publish the zero-coverage reset marker."""
        self._grid.reset()
        self._latest_pose = None
        self._metrics_publisher.publish(Float32(data=0.0))
        return response

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self._latest_pose = (
            position.x,
            position.y,
            self._yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            ),
        )

    def _on_scan(self, message: LaserScan) -> None:
        if self._latest_pose is None:
            return

        self._grid.update_scan(
            self._latest_pose,
            np.asarray(message.ranges, dtype=float),
            message.angle_min,
            message.angle_increment,
            message.range_max,
        )
        self._map_publisher.publish(self._grid.to_message(message.header.stamp))
        self._metrics_publisher.publish(Float32(data=self._grid.coverage_fraction()))

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoverageMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
