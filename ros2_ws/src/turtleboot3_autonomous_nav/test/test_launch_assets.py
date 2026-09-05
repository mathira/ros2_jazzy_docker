from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_declares_runtime_dependencies():
    xml = (PACKAGE_ROOT / 'package.xml').read_text()
    for name in (
        'rclpy',
        'sensor_msgs',
        'nav_msgs',
        'geometry_msgs',
        'tf2_ros',
        'ros_gz_interfaces',
    ):
        assert f'<exec_depend>{name}</exec_depend>' in xml


def test_mission_uses_stage4_without_nav_or_slam():
    source = (PACKAGE_ROOT / 'launch' / 'mission.launch.py').read_text()
    assert 'turtlebot3_dqn_stage4.launch.py' in source
    assert 'nav2' not in source.lower()
    assert 'slam' not in source.lower()


@pytest.mark.parametrize('mode,policy', [('mission', 'dqn_explorer'), ('training', 'dqn_trainer')])
def test_launch_has_one_policy_and_controller(mode, policy, monkeypatch):
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.launch_description_sources import get_launch_description_from_python_launch_file
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    monkeypatch.setenv('TURTLEBOT3_MODEL', 'burger')
    description = get_launch_description_from_python_launch_file(
        str(PACKAGE_ROOT / 'launch' / f'{mode}.launch.py'))
    context = LaunchContext()
    context.launch_configurations['model_path'] = '/tmp/test-policy.pt'
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    executables = [node.node_executable for node in nodes]
    assert executables.count('safe_motion_controller') == 1
    assert executables.count(policy) == 1
    assert ('dqn_trainer' if mode == 'mission' else 'dqn_explorer') not in executables
    assert {'coverage_mapper', 'observation_builder'} <= set(executables)
    assert context.launch_configurations['use_sim_time'] == 'true'
    if mode == 'training':
        assert context.launch_configurations['use_gui'] == 'false'
        context.launch_configurations['episodes'] = '7'
        trainer = nodes[executables.index('dqn_trainer')]
        assert evaluate_parameters(context, trainer._Node__parameters)[-1]['max_episodes'] == 7
    else:
        context.launch_configurations['max_steps'] = '12'
        context.launch_configurations['target_coverage'] = '0.4'
        explorer = nodes[executables.index('dqn_explorer')]
        params = evaluate_parameters(context, explorer._Node__parameters)[-1]
        assert params['max_steps'] == 12
        assert params['target_coverage'] == 0.4
        rviz = nodes[executables.index('rviz2')]
        context.launch_configurations['use_rviz'] = 'false'
        assert not rviz.condition.evaluate(context)


def test_official_stage4_gui_is_gated_without_removing_server(monkeypatch):
    pytest.importorskip('launch_ros')
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    from launch import LaunchContext
    from launch.actions import GroupAction, IncludeLaunchDescription
    from turtleboot3_autonomous_nav.stage4_launch import Stage4LaunchSource

    try:
        share = get_package_share_directory('turtlebot3_gazebo')
    except PackageNotFoundError:
        pytest.skip('Official TurtleBot3 simulation sources are not installed')
    monkeypatch.setenv('TURTLEBOT3_MODEL', 'burger')
    context = LaunchContext()
    context.launch_configurations['use_gui'] = 'false'
    description = Stage4LaunchSource(str(Path(share) / 'launch' /
        'turtlebot3_dqn_stage4.launch.py')).get_launch_description(context)
    gui = [action for action in description.entities if isinstance(action, GroupAction)]
    assert len(gui) == 1
    assert not gui[0].condition.evaluate(context)
    context.launch_configurations['use_gui'] = 'true'
    assert gui[0].condition.evaluate(context)
    # World server, robot state publisher, and robot spawn stay included.
    assert sum(isinstance(action, IncludeLaunchDescription)
               for action in description.entities) == 3
