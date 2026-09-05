"""DQN training over resettable Gazebo exploration episodes.

The reward, termination, checkpoint, and replay-update helpers deliberately
have no ROS dependency.  This keeps the learning contract testable on a host
that does not have a ROS installation; :func:`main` contains the ROS adapter.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any

from turtleboot3_autonomous_nav.dqn import DQNPolicy, ReplayBuffer, save_checkpoint
from turtleboot3_autonomous_nav.reset_provenance import (
    is_post_reset_timestamp,
    source_timestamp_ns,
)


@dataclass(frozen=True)
class TrainerConfig:
    """All trainer limits, reward weights, and DQN schedule parameters."""

    observation_size: int = 86
    action_count: int = 6
    max_steps: int = 500
    max_episodes: int = 100
    target_coverage: float = 0.75
    stall_limit: int = 50
    map_cell_count: int = 160_000
    frontier_threshold: float = 0.50
    new_cell_reward: float = 0.10
    frontier_bonus: float = 0.50
    step_penalty: float = 0.01
    no_progress_penalty: float = 0.10
    intervention_penalty: float = 1.00
    recovery_penalty: float = 0.50
    replay_capacity: int = 100_000
    batch_size: int = 32
    gamma: float = 0.99
    learning_rate: float = 0.001
    target_sync_steps: int = 1_000
    epsilon_start: float = 1.00
    epsilon_min: float = 0.05
    epsilon_decay: float = 0.995
    evaluation_interval: int = 10
    evaluation_episodes: int = 3
    reset_timeout_seconds: float = 10.0

    def __post_init__(self) -> None:
        for name in (
            "observation_size",
            "action_count",
            "max_steps",
            "max_episodes",
            "stall_limit",
            "map_cell_count",
            "replay_capacity",
            "batch_size",
            "target_sync_steps",
            "evaluation_interval",
            "evaluation_episodes",
        ):
            if int(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be positive")
        if not 0.0 < self.target_coverage <= 1.0:
            raise ValueError("target_coverage must be in (0.0, 1.0]")
        if not 0.0 <= self.frontier_threshold <= 1.0:
            raise ValueError("frontier_threshold must be in [0.0, 1.0]")
        if not 0.0 <= self.epsilon_min <= self.epsilon_start <= 1.0:
            raise ValueError("epsilon bounds must be in [0.0, 1.0]")
        if not 0.0 < self.epsilon_decay <= 1.0:
            raise ValueError("epsilon_decay must be in (0.0, 1.0]")
        if not 0.0 < self.gamma <= 1.0 or self.learning_rate <= 0.0:
            raise ValueError("gamma and learning_rate must be positive and bounded")


class EpisodeResetGate:
    """Accept only data received after both reset operations acknowledge.

    The mapper supplies a reset epoch and system-time cutoff in its reset ACK.
    A new episode needs later DDS publication timestamps for odometry and scan,
    so callback delivery order and simulation-clock rewinds cannot admit old data.
    """

    def __init__(self) -> None:
        self._phase = "idle"
        self._cutoff_ns: int | None = None
        self._epoch: int | None = None
        self._fresh = {"odom": False, "scan": False}

    def begin_reset(self) -> None:
        """Start a reset transaction without accepting any sensor data."""
        self._phase = "waiting_for_world"

    def world_reset_succeeded(self) -> None:
        """Advance only after the asynchronous Gazebo acknowledgement succeeds."""
        if self._phase != "waiting_for_world":
            raise RuntimeError("world reset acknowledgement is out of order")
        self._phase = "waiting_for_mapper"

    def mapper_reset_succeeded(self, cutoff_ns: int, epoch: int) -> None:
        """Arm the gate with provenance supplied by the mapper reset ACK."""
        if self._phase != "waiting_for_mapper":
            raise RuntimeError("mapper reset acknowledgement is out of order")
        if int(cutoff_ns) < 0 or int(epoch) < 1:
            raise ValueError("mapper reset provenance must be non-negative and current")
        self._cutoff_ns = int(cutoff_ns)
        self._epoch = int(epoch)
        self._fresh = {"odom": False, "scan": False}
        self._phase = "waiting_for_fresh_data"

    def record_sensor(self, name: str, stamp_ns: int) -> bool:
        """Record a post-ACK required sensor timestamp and return acceptance."""
        if name not in self._fresh:
            raise ValueError(f"unknown required sensor: {name}")
        if self._phase != "waiting_for_fresh_data" or not self.accepts_timestamp(
            stamp_ns
        ):
            return False
        self._fresh[name] = True
        return True

    def accepts_timestamp(self, stamp_ns: int) -> bool:
        """Keep filtering old publications even after the episode has started."""
        return (
            self._phase in ("waiting_for_fresh_data", "running")
            and self._cutoff_ns is not None
            and is_post_reset_timestamp(stamp_ns, self._cutoff_ns)
        )

    @property
    def ready(self) -> bool:
        """Whether an episode can begin with isolated map and sensor state."""
        return self._phase == "waiting_for_fresh_data" and all(self._fresh.values())

    def start_episode(self) -> None:
        """Consume a ready gate so later callbacks cannot reinitialize the episode."""
        if not self.ready:
            raise RuntimeError("cannot start an episode before reset data is fresh")
        self._phase = "running"


def parse_reset_provenance(message: str) -> tuple[int, int]:
    """Parse the mapper-issued reset cutoff and epoch from a Trigger response."""
    try:
        payload = json.loads(message)
        cutoff_ns = payload["cutoff_ns"]
        epoch = payload["epoch"]
    except (TypeError, KeyError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(
            "mapper reset acknowledgement lacks valid provenance"
        ) from error
    if (
        isinstance(cutoff_ns, bool)
        or isinstance(epoch, bool)
        or not isinstance(cutoff_ns, int)
        or not isinstance(epoch, int)
        or cutoff_ns < 0
        or epoch < 1
    ):
        raise ValueError("mapper reset acknowledgement has invalid provenance")
    return cutoff_ns, epoch


def episode_reward(
    new_cells: int,
    reached_frontier: bool,
    safety_intervention: bool,
    recovery_active: bool,
    no_progress: bool = False,
    config: TrainerConfig | None = None,
) -> float:
    """Return the reward for one control step from observable episode events."""
    if int(new_cells) < 0:
        raise ValueError("new_cells cannot be negative")
    weights = config or TrainerConfig()
    reward = float(new_cells) * weights.new_cell_reward - weights.step_penalty
    if reached_frontier:
        reward += weights.frontier_bonus
    if no_progress:
        reward -= weights.no_progress_penalty
    if safety_intervention:
        reward -= weights.intervention_penalty
    if recovery_active:
        reward -= weights.recovery_penalty
    return reward


def episode_end_reason(
    steps: int, coverage: float, stalled_steps: int, config: TrainerConfig
) -> str | None:
    """Return the terminal condition reached by an episode, if any."""
    if steps >= config.max_steps:
        return "max_steps"
    if coverage >= config.target_coverage:
        return "target_coverage"
    if stalled_steps >= config.stall_limit:
        return "stalled"
    return None


def configure_reset_all(request: Any) -> None:
    """Configure a ``ControlWorld`` request for Gazebo's complete reset.

    ``ros_gz_interfaces/srv/ControlWorld`` wraps a ``gz.msgs.WorldControl``;
    the reset-all flag is therefore ``request.world_control.reset.all``.
    """
    try:
        request.world_control.reset.all = True
    except AttributeError as error:
        raise TypeError("ControlWorld request lacks world_control.reset.all") from error


def save_if_improved(
    checkpoint_path: str | Path,
    metrics_path: str | Path,
    policy: DQNPolicy,
    config: Mapping[str, Any],
    metrics: Mapping[str, Any],
    best_mean_coverage: float,
) -> bool:
    """Persist an evaluation checkpoint only if mean coverage strictly improves."""
    mean_coverage = float(metrics.get("mean_coverage", float("nan")))
    if not math.isfinite(mean_coverage):
        raise ValueError("metrics.mean_coverage must be finite")
    if mean_coverage <= float(best_mean_coverage):
        return False

    checkpoint = Path(checkpoint_path)
    metric_file = Path(metrics_path)
    save_checkpoint(checkpoint, policy, config, metrics)
    metric_file.parent.mkdir(parents=True, exist_ok=True)
    metric_file.write_text(
        json.dumps(
            {"config": dict(config), "metrics": dict(metrics)}, indent=2, sort_keys=True
        )
        + "\n",
        encoding="utf-8",
    )
    return True


def optimize_replay(
    policy: DQNPolicy,
    replay: ReplayBuffer,
    batch_size: int,
    gamma: float,
    learning_rate: float,
) -> float | None:
    """Apply one batched TD update to the policy and return its mean loss.

    The package intentionally keeps its small DQN dependency-free.  This is a
    direct back-propagation update over the serializable dense-layer state,
    while target values continue to come from the copied target network.
    """
    if len(replay) < batch_size:
        return None
    if not 0.0 < float(gamma) <= 1.0 or float(learning_rate) <= 0.0:
        raise ValueError("gamma and learning_rate must be positive and bounded")

    state = policy.state_dict()
    layers = state["layers"]
    gradients = _zero_gradients(layers)
    total_loss = 0.0
    transitions = replay.sample(batch_size)
    for transition in transitions:
        activations, pre_activations = _forward_with_activations(
            layers, transition.state
        )
        values = activations[-1]
        target = transition.reward
        if not transition.done:
            target += float(gamma) * max(policy.target_q_values(transition.next_state))
        error = values[transition.action] - target
        total_loss += error * error
        _accumulate_gradients(
            layers,
            gradients,
            activations,
            pre_activations,
            transition.action,
            error,
        )

    scale = float(learning_rate) / float(batch_size)
    for layer, gradient in zip(layers, gradients):
        for output_index, row in enumerate(layer["weights"]):
            for input_index in range(len(row)):
                row[input_index] -= (
                    scale * gradient["weights"][output_index][input_index]
                )
            layer["bias"][output_index] -= scale * gradient["bias"][output_index]
    policy.load_state_dict(state)
    return total_loss / float(batch_size)


def _zero_gradients(layers: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "weights": [[0.0 for _ in row] for row in layer["weights"]],
            "bias": [0.0 for _ in layer["bias"]],
        }
        for layer in layers
    ]


def _forward_with_activations(
    layers: Sequence[Mapping[str, Any]], values: Sequence[float]
) -> tuple[list[list[float]], list[list[float]]]:
    activations = [[float(value) for value in values]]
    pre_activations: list[list[float]] = []
    for layer_index, layer in enumerate(layers):
        current = activations[-1]
        pre_activation = [
            float(bias)
            + sum(float(weight) * value for weight, value in zip(row, current))
            for row, bias in zip(layer["weights"], layer["bias"])
        ]
        pre_activations.append(pre_activation)
        activations.append(
            pre_activation
            if layer_index == len(layers) - 1
            else [max(0.0, value) for value in pre_activation]
        )
    return activations, pre_activations


def _accumulate_gradients(
    layers: Sequence[Mapping[str, Any]],
    gradients: list[dict[str, Any]],
    activations: list[list[float]],
    pre_activations: list[list[float]],
    action: int,
    error: float,
) -> None:
    deltas = [0.0] * len(activations[-1])
    deltas[action] = error
    for layer_index in range(len(layers) - 1, -1, -1):
        layer = layers[layer_index]
        gradient = gradients[layer_index]
        previous = activations[layer_index]
        for output_index, delta in enumerate(deltas):
            gradient["bias"][output_index] += delta
            for input_index, value in enumerate(previous):
                gradient["weights"][output_index][input_index] += delta * value
        if layer_index == 0:
            return
        prior_deltas = [
            sum(
                float(row[input_index]) * delta
                for row, delta in zip(layer["weights"], deltas)
            )
            for input_index in range(len(previous))
        ]
        deltas = [
            delta if pre_activations[layer_index - 1][index] > 0.0 else 0.0
            for index, delta in enumerate(prior_deltas)
        ]


def main(args: list[str] | None = None) -> None:
    """Run the ROS 2 adapter that collects and trains on Gazebo episodes."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from ros_gz_interfaces.srv import ControlWorld
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32
    from std_srvs.srv import Trigger

    class DQNTrainer(Node):
        """Collect asynchronous ROS observations into bounded DQN episodes."""

        def __init__(self) -> None:
            super().__init__("dqn_trainer")
            self._declare_parameters()
            self._config = TrainerConfig(
                **{
                    field: self.get_parameter(field).value
                    for field in TrainerConfig.__dataclass_fields__
                }
            )
            self._policy = DQNPolicy(
                self._config.observation_size, self._config.action_count
            )
            self._replay = ReplayBuffer(self._config.replay_capacity)
            self._action_publisher = self.create_publisher(
                Int32, "/exploration_action", 10
            )
            self._metrics_publisher = self.create_publisher(
                Float32MultiArray, "/training_metrics", 10
            )
            self.create_subscription(
                Float32MultiArray, "/dqn_observation", self._on_observation, 10
            )
            self.create_subscription(
                Float32, "/coverage_metrics", self._on_coverage, 10
            )
            self.create_subscription(
                Bool, "/safety_intervention", self._on_intervention, 10
            )
            self.create_subscription(Bool, "/recovery_active", self._on_recovery, 10)
            self.create_subscription(Odometry, "/odom", self._on_odometry, 10)
            self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
            self._world_control = self.create_client(ControlWorld, "/world/dqn/control")
            self._mapper_reset = self.create_client(Trigger, "/coverage_mapper/reset")

            self._coverage = 0.0
            self._coverage_baseline = 0.0
            self._previous_state: tuple[float, ...] | None = None
            self._previous_action: int | None = None
            self._steps = 0
            self._stalled_steps = 0
            self._episode_reward = 0.0
            self._intervention_seen = False
            self._recovery_seen = False
            self._intervention_active = False
            self._recovery_active = False
            self._reset_gate = EpisodeResetGate()
            self._running_episode = False
            self._is_evaluation = False
            self._training_episodes = 0
            self._total_episodes = 0
            self._evaluation_coverages: list[float] = []
            self._best_mean_coverage = float("-inf")
            self._global_steps = 0
            self._model_directory = Path(
                str(self.declare_parameter("model_directory", "models").value)
            )
            self._request_world_reset()

        def _declare_parameters(self) -> None:
            defaults = TrainerConfig()
            for field, value in asdict(defaults).items():
                self.declare_parameter(field, value)

        def _on_coverage(self, message: Float32, info: Any) -> None:
            if not self._reset_gate.accepts_timestamp(source_timestamp_ns(info)):
                return
            coverage = float(message.data)
            if not math.isfinite(coverage):
                return
            coverage = min(max(coverage, 0.0), 1.0)
            if self._running_episode:
                self._coverage = coverage

        def _on_intervention(self, message: Bool, info: Any) -> None:
            if not self._reset_gate.accepts_timestamp(source_timestamp_ns(info)):
                return
            active = bool(message.data)
            self._intervention_seen |= active and not self._intervention_active
            self._intervention_active = active

        def _on_recovery(self, message: Bool, info: Any) -> None:
            if not self._reset_gate.accepts_timestamp(source_timestamp_ns(info)):
                return
            active = bool(message.data)
            self._recovery_seen |= active and not self._recovery_active
            self._recovery_active = active

        def _on_odometry(self, message: Odometry, info: Any) -> None:
            stamp_ns = source_timestamp_ns(info)
            if self._reset_gate.record_sensor("odom", stamp_ns):
                self._begin_episode_when_reset_data_is_fresh()

        def _on_scan(self, message: LaserScan, info: Any) -> None:
            stamp_ns = source_timestamp_ns(info)
            if self._reset_gate.record_sensor("scan", stamp_ns):
                self._begin_episode_when_reset_data_is_fresh()

        def _begin_episode_when_reset_data_is_fresh(self) -> None:
            if not self._reset_gate.ready:
                return
            self._reset_gate.start_episode()
            self._running_episode = True
            self._coverage = 0.0
            self._coverage_baseline = 0.0
            self._previous_state = None
            self._previous_action = None
            self._steps = 0
            self._stalled_steps = 0
            self._episode_reward = 0.0
            self._intervention_seen = False
            self._recovery_seen = False
            self.get_logger().info(
                f"Started {'evaluation' if self._is_evaluation else 'training'} "
                f"episode {self._total_episodes + 1} after reset ACKs and fresh "
                "odometry and scan."
            )

        def _on_observation(self, message: Float32MultiArray, info: Any) -> None:
            if not self._running_episode or not self._reset_gate.accepts_timestamp(
                source_timestamp_ns(info)
            ):
                return
            try:
                observation = tuple(float(value) for value in message.data)
                if len(observation) != self._config.observation_size or not all(
                    math.isfinite(value) for value in observation
                ):
                    raise ValueError("malformed observation")
            except (TypeError, ValueError) as error:
                self.get_logger().warning(f"Rejected DQN observation: {error}")
                return

            if self._previous_state is not None and self._previous_action is not None:
                self._record_transition(observation)
                if not self._running_episode:
                    return
            epsilon = 0.0 if self._is_evaluation else self._epsilon()
            action = self._policy.select_training_action(observation, epsilon)
            self._action_publisher.publish(Int32(data=action))
            self._previous_state = observation
            self._previous_action = action

        def _record_transition(self, next_state: tuple[float, ...]) -> None:
            assert (
                self._previous_state is not None and self._previous_action is not None
            )
            new_cells = max(
                0,
                int(
                    round(
                        (self._coverage - self._coverage_baseline)
                        * self._config.map_cell_count
                    )
                ),
            )
            self._coverage_baseline = self._coverage
            no_progress = new_cells == 0
            self._stalled_steps = self._stalled_steps + 1 if no_progress else 0
            reached_frontier = (
                new_cells > 0
                and max(next_state[12:20], default=0.0)
                >= self._config.frontier_threshold
            )
            reward = episode_reward(
                new_cells,
                reached_frontier,
                self._intervention_seen,
                self._recovery_seen,
                no_progress,
                self._config,
            )
            self._intervention_seen = False
            self._recovery_seen = False
            self._steps += 1
            self._global_steps += 1
            self._episode_reward += reward
            reason = episode_end_reason(
                self._steps, self._coverage, self._stalled_steps, self._config
            )
            done = reason is not None
            self._replay.add(
                self._previous_state, self._previous_action, reward, next_state, done
            )
            if not self._is_evaluation:
                optimize_replay(
                    self._policy,
                    self._replay,
                    self._config.batch_size,
                    self._config.gamma,
                    self._config.learning_rate,
                )
                if self._global_steps % self._config.target_sync_steps == 0:
                    self._policy.copy_target_network()
            self._publish_metrics()
            if done:
                self._finish_episode(reason)

        def _finish_episode(self, reason: str) -> None:
            self._running_episode = False
            self._total_episodes += 1
            if self._is_evaluation:
                self._evaluation_coverages.append(self._coverage)
                if len(self._evaluation_coverages) >= self._config.evaluation_episodes:
                    mean_coverage = sum(self._evaluation_coverages) / len(
                        self._evaluation_coverages
                    )
                    metrics = {
                        "episode": self._total_episodes,
                        "mean_coverage": mean_coverage,
                        "evaluation_episodes": len(self._evaluation_coverages),
                        "global_steps": self._global_steps,
                    }
                    if save_if_improved(
                        self._model_directory / "best.pt",
                        self._model_directory / "best.metrics.json",
                        self._policy,
                        asdict(self._config),
                        metrics,
                        self._best_mean_coverage,
                    ):
                        self._best_mean_coverage = mean_coverage
                    self._evaluation_coverages.clear()
                    self._is_evaluation = False
            else:
                self._training_episodes += 1
                if self._training_episodes % self._config.evaluation_interval == 0:
                    self._is_evaluation = True
            self.get_logger().info(
                f"Episode ended: {reason}; coverage={self._coverage:.3f}; "
                f"reward={self._episode_reward:.3f}"
            )
            if (
                self._training_episodes >= self._config.max_episodes
                and not self._is_evaluation
            ):
                self.get_logger().info("Configured training episode limit reached.")
                return
            self._request_world_reset()

        def _request_world_reset(self) -> None:
            self._reset_gate.begin_reset()
            if not self._world_control.wait_for_service(
                timeout_sec=self._config.reset_timeout_seconds
            ):
                self.get_logger().error(
                    "Gazebo ControlWorld service /world/dqn/control is unavailable; "
                    "training cannot continue safely."
                )
                rclpy.shutdown()
                return
            request = ControlWorld.Request()
            configure_reset_all(request)
            future = self._world_control.call_async(request)
            future.add_done_callback(self._on_world_reset)

        def _on_world_reset(self, future: Any) -> None:
            try:
                response = future.result()
            except Exception as error:
                self.get_logger().error(f"Gazebo world reset failed: {error}")
                rclpy.shutdown()
                return
            if not response.success:
                self.get_logger().error(
                    f"Gazebo world reset was rejected: {response.message}"
                )
                rclpy.shutdown()
                return
            self._reset_gate.world_reset_succeeded()
            self._request_mapper_reset()

        def _request_mapper_reset(self) -> None:
            if not self._mapper_reset.wait_for_service(
                timeout_sec=self._config.reset_timeout_seconds
            ):
                self.get_logger().error(
                    "Coverage mapper reset service /coverage_mapper/reset is unavailable; "
                    "training cannot continue with mixed episode coverage."
                )
                rclpy.shutdown()
                return
            future = self._mapper_reset.call_async(Trigger.Request())
            future.add_done_callback(self._on_mapper_reset)

        def _on_mapper_reset(self, future: Any) -> None:
            try:
                response = future.result()
            except Exception as error:
                self.get_logger().error(f"Coverage mapper reset failed: {error}")
                rclpy.shutdown()
                return
            if not response.success:
                self.get_logger().error(
                    f"Coverage mapper reset was rejected: {response.message}"
                )
                rclpy.shutdown()
                return
            try:
                cutoff_ns, epoch = parse_reset_provenance(response.message)
            except ValueError as error:
                self.get_logger().error(f"Invalid mapper reset provenance: {error}")
                rclpy.shutdown()
                return
            self._reset_gate.mapper_reset_succeeded(cutoff_ns, epoch)
            self._coverage = 0.0
            self._coverage_baseline = 0.0

        def _epsilon(self) -> float:
            decayed = self._config.epsilon_start * (
                self._config.epsilon_decay**self._training_episodes
            )
            return max(self._config.epsilon_min, decayed)

        def _publish_metrics(self) -> None:
            self._metrics_publisher.publish(
                Float32MultiArray(
                    data=[
                        float(self._total_episodes + 1),
                        float(self._steps),
                        self._coverage,
                        self._episode_reward,
                        self._epsilon(),
                        (
                            self._best_mean_coverage
                            if math.isfinite(self._best_mean_coverage)
                            else 0.0
                        ),
                    ]
                )
            )

    rclpy.init(args=args)
    node = DQNTrainer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
