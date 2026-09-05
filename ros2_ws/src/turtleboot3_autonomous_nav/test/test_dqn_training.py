import random

import pytest

from turtleboot3_autonomous_nav.dqn import (
    DQNPolicy,
    ReplayBuffer,
    load_checkpoint,
    save_checkpoint,
)
from turtleboot3_autonomous_nav.dqn_explorer import greedy_action_for_observation


def test_checkpoint_round_trip_preserves_action_space(tmp_path):
    """A saved model retains its discrete action-space metadata."""
    policy = DQNPolicy(observation_size=10, action_count=6)
    observation = [0.25] * 10
    expected_values = policy.q_values(observation)

    save_checkpoint(
        tmp_path / 'best.pt',
        policy,
        {'epsilon': 0.0},
        {'coverage': 0.5},
    )

    loaded, _, metrics = load_checkpoint(tmp_path / 'best.pt')

    assert loaded.action_count == 6
    assert metrics['coverage'] == 0.5
    assert loaded.q_values(observation) == expected_values


def test_replay_buffer_retains_only_its_most_recent_capacity():
    """Old experiences must be evicted instead of growing replay without bound."""
    replay = ReplayBuffer(capacity=2)

    for action in range(3):
        replay.add([action], action, float(action), [action + 1], False)

    retained_actions = {transition.action for transition in replay.sample(2)}

    assert len(replay) == 2
    assert retained_actions == {1, 2}


def test_training_selection_uses_epsilon_to_choose_exploration_or_greedy_action():
    """Epsilon zero is greedy, while epsilon one uses the supplied random action."""
    policy = DQNPolicy(observation_size=3, action_count=4)
    observation = [0.1, -0.2, 0.3]
    greedy_action = policy.select_action(observation)
    exploratory_action = (greedy_action + 1) % policy.action_count

    class ForcedExploration:
        def random(self):
            return 0.0

        def randrange(self, stop):
            assert stop == policy.action_count
            return exploratory_action

    assert policy.select_training_action(observation, epsilon=0.0) == greedy_action
    assert (
        policy.select_training_action(
            observation, epsilon=1.0, rng=ForcedExploration()
        )
        == exploratory_action
    )


def test_copy_target_network_replaces_target_weights_with_online_weights():
    """Target synchronization must discard stale target-network values."""
    policy = DQNPolicy(observation_size=2, action_count=2)
    observation = [1.0, -1.0]
    state = policy.state_dict()
    state['target_layers'][-1]['bias'][0] += 100.0
    policy.load_state_dict(state)

    assert policy.target_q_values(observation) != policy.q_values(observation)

    policy.copy_target_network()

    assert policy.target_q_values(observation) == policy.q_values(observation)


def test_inference_rejects_an_observation_with_the_wrong_dimension():
    """A malformed observation must not reach the network's first layer."""
    policy = DQNPolicy(observation_size=10, action_count=6)

    with pytest.raises(ValueError, match='expected observation size 10'):
        policy.select_action([0.0] * 9)


def test_explorer_adapter_uses_greedy_policy_inference():
    """Exploration deployment must not accidentally retain training epsilon."""
    policy = DQNPolicy(observation_size=3, action_count=4)
    observation = [0.2, 0.1, -0.5]

    assert greedy_action_for_observation(policy, observation) == policy.select_action(
        observation
    )
