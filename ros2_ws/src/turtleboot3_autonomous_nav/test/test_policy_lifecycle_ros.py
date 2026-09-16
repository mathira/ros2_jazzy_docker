"""Exercise policy adapters with real ROS messages and controlled service transport."""

import json
import tempfile
from dataclasses import replace
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.task import Future
from ros_gz_interfaces.srv import ControlWorld
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension
from std_srvs.srv import SetBool, Trigger

from turtleboot3_autonomous_nav import dqn_explorer, dqn_trainer
from turtleboot3_autonomous_nav.dqn import DQNPolicy, save_checkpoint
from turtleboot3_autonomous_nav.control import POLICY_ACTION_COUNT
from turtleboot3_autonomous_nav.observation import OBSERVATION_SIZE


class ServiceTransport:
    def __init__(self):
        self.ready = True
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def wait_for_service(self, **kwargs):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        return future

    def reply(self, response):
        self.calls[-1][1].set_result(response)


@pytest.fixture
def transport(monkeypatch):
    clients = {}

    def create_client(node, service_type, name, *args, **kwargs):
        return clients.setdefault(name, ServiceTransport())

    monkeypatch.setattr(Node, 'create_client', create_client)
    return clients


def provenance(response_type, cutoff=1_000, epoch=1):
    return response_type(success=True, message=json.dumps({'cutoff_ns': cutoff, 'epoch': epoch}))


def deliver(stamp=2_000):
    return {'source_timestamp': stamp}


def observation(epoch=1):
    message = Float32MultiArray(data=[0.0] * OBSERVATION_SIZE)
    message.layout.dim = [MultiArrayDimension(
        label=f'episode:{epoch}', size=OBSERVATION_SIZE, stride=OBSERVATION_SIZE)]
    return message


def complete_trainer_reset(trainer, clients):
    trainer._poll_reset()
    control = clients['/safe_motion_controller/enable']
    assert control.calls[-1][0].data is False
    control.reply(provenance(SetBool.Response))
    trainer._poll_reset()
    clients['/world/dqn/control'].reply(ControlWorld.Response(success=True))
    trainer._poll_reset()
    clients['/coverage_mapper/reset'].reply(provenance(Trigger.Response))
    trainer._poll_reset()
    clients['/observation_builder/reset'].reply(provenance(Trigger.Response, 1_100, 7))
    trainer._poll_reset()
    assert control.calls[-1][0].data is True
    control.reply(provenance(SetBool.Response, 1_200, 2))


def test_trainer_stops_before_reset_and_requires_current_observation_epoch(monkeypatch, transport):
    def exercise(trainer):
        complete_trainer_reset(trainer, transport)
        trainer._on_odometry(Odometry(), deliver())
        trainer._on_scan(LaserScan(), deliver())
        assert trainer._running_episode
        trainer._on_observation(observation(epoch=6), deliver())
        assert trainer._previous_state is None
        trainer._on_observation(observation(epoch=7), deliver())
        assert trainer._previous_state is not None

    monkeypatch.setattr(rclpy, 'spin', exercise)
    dqn_trainer.main(['--ros-args', '-p', f'model_directory:={tempfile.mkdtemp()}',
                     '-p', 'resume:=false'])


@pytest.mark.parametrize('phase', ['controller_service', 'disable_reply', 'world_reply', 'mapper_reply',
                                    'builder_reply', 'enable_reply', 'sensors', 'observation'])
def test_trainer_reset_phases_have_wall_clock_deadlines(monkeypatch, transport, phase):
    wall_time = [time.monotonic_ns()]
    monkeypatch.setattr(time, 'monotonic_ns', lambda: wall_time[0])

    def exercise(trainer):
        control = transport['/safe_motion_controller/enable']
        if phase == 'controller_service':
            control.ready = False
        trainer._poll_reset()
        if phase not in ('controller_service', 'disable_reply'):
            control.reply(provenance(SetBool.Response))
            trainer._poll_reset()
        if phase not in ('controller_service', 'disable_reply', 'world_reply'):
            transport['/world/dqn/control'].reply(ControlWorld.Response(success=True))
            trainer._poll_reset()
        if phase in ('builder_reply', 'enable_reply', 'sensors', 'observation'):
            transport['/coverage_mapper/reset'].reply(provenance(Trigger.Response))
            trainer._poll_reset()
        if phase in ('enable_reply', 'sensors', 'observation'):
            transport['/observation_builder/reset'].reply(provenance(Trigger.Response, 1_100, 7))
            trainer._poll_reset()
        if phase in ('sensors', 'observation'):
            control.reply(provenance(SetBool.Response, 1_200, 2))
        if phase == 'observation':
            trainer._on_odometry(Odometry(), deliver())
            trainer._on_scan(LaserScan(), deliver())
        # A reset RPC is retried before it is fatal, so exhaust the budget;
        # the sensor phases have no retries and fail on the first deadline.
        for _ in range(trainer._config.reset_retry_limit + 1):
            wall_time[0] += 11_000_000_000
            trainer._poll_reset()
        assert trainer._failed
        assert not trainer._running_episode
        assert trainer._previous_state is None
        # Failure must request a stop and continue suppressing old callbacks.
        control.ready = True
        trainer._poll_reset()
        assert control.calls[-1][0].data is False
        trainer._on_observation(observation(epoch=7), deliver())
        assert trainer._previous_state is None

    monkeypatch.setattr(rclpy, 'spin', exercise)
    with pytest.raises(RuntimeError, match='Training aborted'):
        dqn_trainer.main(['--ros-args', '-p', f'model_directory:={tempfile.mkdtemp()}',
                     '-p', 'resume:=false'])


def test_trainer_episode_limit_requests_acknowledged_stop(monkeypatch, transport):
    def exercise(trainer):
        complete_trainer_reset(trainer, transport)
        trainer._config = replace(trainer._config, max_steps=1, max_episodes=1)
        trainer._on_odometry(Odometry(), deliver())
        trainer._on_scan(LaserScan(), deliver())
        trainer._on_observation(observation(epoch=7), deliver())
        trainer._on_observation(observation(epoch=7), deliver())
        assert not trainer._running_episode
        trainer._poll_reset()
        control = transport['/safe_motion_controller/enable']
        assert control.calls[-1][0].data is False
        assert len(transport['/world/dqn/control'].calls) == 1

    monkeypatch.setattr(rclpy, 'spin', exercise)
    dqn_trainer.main(['--ros-args', '-p', f'model_directory:={tempfile.mkdtemp()}',
                     '-p', 'resume:=false'])


def test_trainer_reset_failure_exits_unsuccessfully_after_stop_ack(monkeypatch, transport):
    def exercise(trainer):
        trainer._poll_reset()
        control = transport['/safe_motion_controller/enable']
        control.reply(provenance(SetBool.Response))
        trainer._poll_reset()
        transport['/world/dqn/control'].reply(ControlWorld.Response(success=False))
        trainer._poll_reset()
        assert control.calls[-1][0].data is False
        control.reply(provenance(SetBool.Response))
        assert not rclpy.ok()

    monkeypatch.setattr(rclpy, 'spin', exercise)
    with pytest.raises(RuntimeError, match='Training aborted'):
        dqn_trainer.main(['--ros-args', '-p', f'model_directory:={tempfile.mkdtemp()}',
                     '-p', 'resume:=false'])


@pytest.mark.parametrize('model_path', ['', '   ', '/missing/policy.pt'])
def test_explorer_rejects_missing_model_before_activation(monkeypatch, model_path):
    monkeypatch.setattr(rclpy, 'spin', lambda _: None)
    try:
        with pytest.raises((ValueError, FileNotFoundError)):
            dqn_explorer.main(['--ros-args', '-p', f'model_path:={json.dumps(model_path)}'])
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize('reason', ['max_steps', 'target_coverage'])
def test_mission_termination_disables_controller_and_suppresses_later_actions(
    monkeypatch, transport, tmp_path, reason
):
    model = tmp_path / 'policy.pt'
    save_checkpoint(
        model, DQNPolicy(OBSERVATION_SIZE, POLICY_ACTION_COUNT, seed=1), {}, {}
    )

    def exercise(explorer):
        from turtleboot3_autonomous_nav.safe_motion_controller import SafeMotionController
        controller = SafeMotionController()
        decisions = []
        controller._publish_decision = decisions.append
        explorer._poll_control()
        control = transport['/safe_motion_controller/enable']
        control.reply(controller._on_enable(control.calls[-1][0], SetBool.Response()))
        actions = []
        def publish_action(message):
            actions.append(message)
            controller._on_action(message, deliver(time.time_ns()))
        explorer._publisher = SimpleNamespace(publish=publish_action)
        controller._on_scan(LaserScan(ranges=[2.0], angle_increment=1.0, range_max=3.5), deliver(time.time_ns()))
        controller._on_odometry(Odometry(), deliver(time.time_ns()))
        explorer._on_observation(observation())
        assert len(actions) == 1
        controller._on_control_timer()
        assert abs(decisions[-1].linear_x) + abs(decisions[-1].angular_z) > 0
        if reason == 'target_coverage':
            explorer._on_coverage(Float32(data=0.8))
        else:
            explorer._on_observation(observation())
        assert explorer._finished
        explorer._poll_control()
        assert control.calls[-1][0].data is False
        control.reply(controller._on_enable(control.calls[-1][0], SetBool.Response()))
        assert decisions[-1].linear_x == decisions[-1].angular_z == 0
        explorer._on_observation(observation())
        assert len(actions) == (1 if reason == 'target_coverage' else 2)
        controller.destroy_node()

    monkeypatch.setattr(rclpy, 'spin', exercise)
    dqn_explorer.main(['--ros-args', '-p', f'model_path:={model}', '-p', 'max_steps:=2'])
