"""Read-only RViz markers for velocity commands and coverage metrics."""

import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class ExplorationVisualizer(Node):
    def __init__(self):
        super().__init__('exploration_visualizer')
        self._velocity = Twist()
        self._coverage = 0.0
        self.create_subscription(Twist, '/cmd_vel', self._on_velocity, 10)
        self.create_subscription(Float32, '/coverage_metrics', self._on_coverage, 10)
        self._publisher = self.create_publisher(MarkerArray, '/exploration_status', 10)
        self.create_timer(0.2, self._publish)

    def _on_velocity(self, message):
        self._velocity = message

    def _on_coverage(self, message):
        self._coverage = message.data

    def _publish(self):
        arrow = Marker()
        arrow.header.frame_id = 'base_footprint'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'cmd_vel'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.03, 0.07, 0.10
        arrow.color.g, arrow.color.a = 1.0, 1.0
        # Arrow shows the command's one-second direction and scaled translation.
        angle = self._velocity.angular.z
        distance = abs(self._velocity.linear.x) * 3.0
        arrow.points = [Point(z=0.15), Point(
            x=distance * math.cos(angle), y=distance * math.sin(angle), z=0.15)]
        label = Marker()
        label.header = arrow.header
        label.ns = 'coverage_metrics'
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.z = 0.75
        label.scale.z = 0.16
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = (f'Coverage: {self._coverage:.2%}\n'
                      f'cmd_vel: {self._velocity.linear.x:.2f} m/s, '
                      f'{self._velocity.angular.z:.2f} rad/s')
        self._publisher.publish(MarkerArray(markers=[arrow, label]))


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
