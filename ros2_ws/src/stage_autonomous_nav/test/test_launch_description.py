from stage_autonomous_nav.launch.cave_navigation import generate_launch_description
from pathlib import Path


def test_cave_navigation_declares_supported_arguments():
    names = {
        action.name
        for action in generate_launch_description().entities
        if hasattr(action, 'name')
    }

    assert {'world', 'map_frame', 'odom_frame', 'base_frame', 'scan_topic', 'rviz'} <= names


def test_nav2_composition_flag_is_a_valid_python_boolean_literal():
    source = Path(__file__).parents[1] / 'stage_autonomous_nav' / 'launch' / 'cave_navigation.py'
    assert "'use_composition': 'False'" in source.read_text()


def test_stage_one_tf_tree_defaults_use_robot_prefixed_frames():
    source = Path(__file__).parents[1] / 'stage_autonomous_nav' / 'launch' / 'cave_navigation.py'
    text = source.read_text()
    assert "DeclareLaunchArgument('odom_frame', default_value='robot_0/odom')" in text
    assert "DeclareLaunchArgument('base_frame', default_value='robot_0/base_link')" in text
