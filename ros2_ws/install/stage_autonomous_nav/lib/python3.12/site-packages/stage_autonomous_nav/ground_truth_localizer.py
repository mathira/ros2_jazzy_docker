"""Publish the map-to-odom transform using Stage ground truth."""

from math import atan2, cos, sin

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
except ModuleNotFoundError:  # Allows the pure conversion helpers to be unit-tested without ROS.
    rclpy = None

from stage_autonomous_nav.transform_math import map_to_odom


def yaw_from_quaternion(x, y, z, w):
    """Return the planar yaw represented by a quaternion."""
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """Return the quaternion representing a rotation about the z axis."""
    return (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0))


if rclpy is not None:

    class GroundTruthLocalizer(Node):
        """Derive map->odom from ground truth and odometry TF."""

        def __init__(self):
            super().__init__('ground_truth_localizer')
            self.declare_parameter('ground_truth_topic', '/ground_truth')
            self.declare_parameter('map_frame', 'map')
            self.declare_parameter('odom_frame', 'odom')
            self.declare_parameter('base_frame', 'base_link')
            self.declare_parameter('publish_rate_hz', 20.0)

            self._ground_truth = None
            self._map_frame = self.get_parameter('map_frame').value
            self._odom_frame = self.get_parameter('odom_frame').value
            self._base_frame = self.get_parameter('base_frame').value
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._tf_broadcaster = TransformBroadcaster(self)
            self._subscription = self.create_subscription(
                Odometry,
                self.get_parameter('ground_truth_topic').value,
                self._ground_truth_callback,
                10,
            )
            publish_rate_hz = self.get_parameter('publish_rate_hz').value
            self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_transform)

        def _ground_truth_callback(self, message):
            self._ground_truth = message

        def _publish_transform(self):
            if self._ground_truth is None:
                return

            try:
                odom_to_base = self._tf_buffer.lookup_transform(
                    self._odom_frame, self._base_frame, Time()
                )
            except TransformException as error:
                self.get_logger().warning(
                    f'Unable to look up {self._odom_frame} -> {self._base_frame}: {error}',
                    throttle_duration_sec=5.0,
                )
                return

            ground_truth_pose = self._ground_truth.pose.pose
            odom_pose = odom_to_base.transform
            map_x, map_y, map_yaw = (
                ground_truth_pose.position.x,
                ground_truth_pose.position.y,
                yaw_from_quaternion(
                    ground_truth_pose.orientation.x,
                    ground_truth_pose.orientation.y,
                    ground_truth_pose.orientation.z,
                    ground_truth_pose.orientation.w,
                ),
            )
            odom_x, odom_y, odom_yaw = (
                odom_pose.translation.x,
                odom_pose.translation.y,
                yaw_from_quaternion(
                    odom_pose.rotation.x,
                    odom_pose.rotation.y,
                    odom_pose.rotation.z,
                    odom_pose.rotation.w,
                ),
            )
            x, y, yaw = map_to_odom(
                map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw
            )

            transform = TransformStamped()
            transform.header.stamp = self._ground_truth.header.stamp
            if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
                transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self._map_frame
            transform.child_frame_id = self._odom_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ) = quaternion_from_yaw(yaw)
            self._tf_broadcaster.sendTransform(transform)


else:

    class GroundTruthLocalizer:
        """Placeholder that explains the missing ROS 2 runtime."""

        def __init__(self):
            raise RuntimeError('GroundTruthLocalizer requires a ROS 2 Python runtime.')


def main(args=None):
    """Run the ground-truth localizer node."""
    if rclpy is None:
        raise RuntimeError('ground_truth_localizer requires a ROS 2 Python runtime.')

    rclpy.init(args=args)
    node = GroundTruthLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
