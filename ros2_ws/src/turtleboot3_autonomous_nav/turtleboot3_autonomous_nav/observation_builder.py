"""ROS adapter that publishes fixed DQN observations from map, scan, and odometry."""

from __future__ import annotations

import json
import time
from typing import Any

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from turtleboot3_autonomous_nav.observation import LOCAL_PATCH_SIZE, build_observation
from turtleboot3_autonomous_nav.reset_provenance import (
    is_post_reset_timestamp,
    source_timestamp_ns,
)


class ObservationBuilder(Node):
    """Publish an observation for each scan once matching map and odometry exist."""

    def __init__(self) -> None:
        super().__init__('observation_builder')
        self._latest_scan: np.ndarray | None = None
        self._map: OccupancyGrid | None = None
        self._position: tuple[float, float] | None = None
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._reset_cutoff_ns: int | None = None
        self._reset_epoch = 0
        self._publisher = self.create_publisher(Float32MultiArray, '/dqn_observation', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(OccupancyGrid, '/coverage_map', self._on_map, 10)
        self.create_subscription(Odometry, '/odom', self._on_odometry, 10)
        self.create_service(Trigger, '/observation_builder/reset', self._on_reset)

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response):
        """Clear cached inputs after the mapper reset and establish an epoch."""
        self._latest_scan = self._map = self._position = None
        self._linear_velocity = self._angular_velocity = 0.0
        self._reset_epoch += 1
        self._reset_cutoff_ns = time.time_ns()
        response.success = True
        response.message = json.dumps({
            'cutoff_ns': self._reset_cutoff_ns, 'epoch': self._reset_epoch,
        })
        return response

    def _accepts(self, info: Any) -> bool:
        return is_post_reset_timestamp(source_timestamp_ns(info), self._reset_cutoff_ns)

    def _on_scan(self, message: LaserScan, info: Any) -> None:
        if not self._accepts(info):
            return
        self._latest_scan = np.asarray(message.ranges, dtype=float)
        self._publish_if_ready()

    def _on_map(self, message: OccupancyGrid, info: Any) -> None:
        if not self._accepts(info):
            return
        self._map = message

    def _on_odometry(self, message: Odometry, info: Any) -> None:
        if not self._accepts(info):
            return
        position = message.pose.pose.position
        self._position = (position.x, position.y)
        self._linear_velocity = message.twist.twist.linear.x
        self._angular_velocity = message.twist.twist.angular.z

    def _publish_if_ready(self) -> None:
        if self._latest_scan is None or self._map is None or self._position is None:
            return
        observation = build_observation(
            self._latest_scan,
            self._local_grid_patch(self._map, self._position),
            self._linear_velocity,
            self._angular_velocity,
        )
        message = Float32MultiArray(data=observation.tolist())
        message.layout.dim = [MultiArrayDimension(
            label=f'episode:{self._reset_epoch}', size=len(observation), stride=len(observation)
        )]
        self._publisher.publish(message)

    @staticmethod
    def _local_grid_patch(
        message: OccupancyGrid, position: tuple[float, float]
    ) -> np.ndarray:
        """Extract an unknown-padded map patch centred on the odometric pose."""
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return np.full((LOCAL_PATCH_SIZE, LOCAL_PATCH_SIZE), -1, dtype=np.int8)

        map_data = np.asarray(message.data, dtype=np.int8)
        if map_data.size != width * height:
            return np.full((LOCAL_PATCH_SIZE, LOCAL_PATCH_SIZE), -1, dtype=np.int8)
        grid = map_data.reshape(height, width)
        origin = message.info.origin.position
        center_x = int(np.floor((position[0] - origin.x) / resolution))
        center_y = int(np.floor((position[1] - origin.y) / resolution))
        patch = np.full((LOCAL_PATCH_SIZE, LOCAL_PATCH_SIZE), -1, dtype=np.int8)
        half = LOCAL_PATCH_SIZE // 2
        for patch_y in range(LOCAL_PATCH_SIZE):
            grid_y = center_y + patch_y - half
            if not 0 <= grid_y < height:
                continue
            for patch_x in range(LOCAL_PATCH_SIZE):
                grid_x = center_x + patch_x - half
                if 0 <= grid_x < width:
                    patch[patch_y, patch_x] = grid[grid_y, grid_x]
        return patch


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObservationBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
