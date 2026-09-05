"""ROS 2 adapter for greedy DQN exploration inference."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from turtleboot3_autonomous_nav.dqn import DQNPolicy, load_checkpoint


def greedy_action_for_observation(
    policy: DQNPolicy, observation: Sequence[float]
) -> int:
    """Select the deployment action without applying training exploration."""
    return policy.select_action(observation)


def main(args: list[str] | None = None) -> None:
    """Run the ROS adapter after loading an optional trained checkpoint."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, Int32

    class DQNExplorer(Node):
        """Convert each complete DQN observation into one discrete action."""

        def __init__(self) -> None:
            super().__init__('dqn_explorer')
            self.declare_parameter('model_path', '')
            self.declare_parameter('observation_size', 86)
            self.declare_parameter('action_count', 6)
            observation_size = int(self.get_parameter('observation_size').value)
            action_count = int(self.get_parameter('action_count').value)
            model_path = str(self.get_parameter('model_path').value)
            self._policy = self._load_policy(
                model_path, observation_size, action_count
            )
            self._publisher = self.create_publisher(Int32, '/exploration_action', 10)
            self.create_subscription(
                Float32MultiArray, '/dqn_observation', self._on_observation, 10
            )

        def _load_policy(
            self, model_path: str, observation_size: int, action_count: int
        ) -> DQNPolicy:
            if not model_path:
                self.get_logger().warning(
                    'No model_path configured; publishing greedy actions from a '
                    'newly initialized policy.'
                )
                return DQNPolicy(observation_size, action_count)

            policy, _, _ = load_checkpoint(Path(model_path))
            if policy.observation_size != observation_size:
                raise ValueError(
                    'checkpoint observation size does not match observation_size '
                    'parameter'
                )
            if policy.action_count != action_count:
                raise ValueError(
                    'checkpoint action count does not match action_count parameter'
                )
            return policy

        def _on_observation(self, message: Float32MultiArray) -> None:
            try:
                action = greedy_action_for_observation(self._policy, message.data)
            except ValueError as error:
                self.get_logger().error(f'Rejected DQN observation: {error}')
                return
            self._publisher.publish(Int32(data=action))

    rclpy.init(args=args)
    node = DQNExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
