from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

from stage_pid_navigation.control import NavigationConfig


LAUNCH_FILE = (
    Path(__file__).parents[1]
    / "stage_pid_navigation"
    / "launch"
    / "pid_navigation.launch.py"
)

EXPECTED_DEFAULTS = {
    "odom_topic": "/odom",
    "scan_topic": "/base_scan",
    "cmd_vel_topic": "/cmd_vel",
    "control_rate": 10.0,
    "kp": 1.8,
    "ki": 0.0,
    "kd": 0.15,
    "integral_limit": 1.0,
    "max_linear_speed": 0.35,
    "max_angular_speed": 1.2,
    "heading_stop_threshold": 0.7,
    "goal_tolerance": 0.15,
    "slowdown_distance": 0.9,
    "stop_distance": 0.35,
    "front_sector_angle": 0.7,
    "require_scan": True,
}

FLOAT_PARAMETERS = {
    "goal_x",
    "goal_y",
    "control_rate",
    "kp",
    "ki",
    "kd",
    "integral_limit",
    "max_linear_speed",
    "max_angular_speed",
    "heading_stop_threshold",
    "goal_tolerance",
    "slowdown_distance",
    "stop_distance",
    "front_sector_angle",
}


def load_launch_description():
    spec = spec_from_file_location("pid_navigation_launch", LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def launch_arguments(description):
    return {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }


def navigator_parameters(description):
    node = next(action for action in description.entities if isinstance(action, Node))
    return vars(node)["_Node__parameters"]


def test_launch_declares_stage_topics_and_goal_arguments():
    description = load_launch_description()
    arguments = launch_arguments(description)

    assert set(arguments) == {"goal_x", "goal_y", *EXPECTED_DEFAULTS}

    node = next(action for action in description.entities if isinstance(action, Node))
    node_fields = vars(node)
    assert node_fields["_Node__package"] == "stage_pid_navigation"
    assert node_fields["_Node__node_executable"] == "pid_navigator"
    parameters = navigator_parameters(description)[0]
    assert {key[0].text for key in parameters} == set(arguments)


@pytest.mark.parametrize("goal_name", ["goal_x", "goal_y"])
def test_goal_launch_arguments_are_required(goal_name):
    description = load_launch_description()
    declaration = launch_arguments(description)[goal_name]

    assert declaration.default_value is None
    with pytest.raises(RuntimeError, match=f'Required launch argument "{goal_name}"'):
        declaration.visit(LaunchContext())


def test_launch_and_control_defaults_match_design():
    description = load_launch_description()
    context = LaunchContext()
    context.launch_configurations.update({"goal_x": "2", "goal_y": "1.5"})
    for name, declaration in launch_arguments(description).items():
        if name not in {"goal_x", "goal_y"}:
            declaration.visit(context)

    evaluated = evaluate_parameters(context, navigator_parameters(description))[0]

    assert {name: evaluated[name] for name in EXPECTED_DEFAULTS} == EXPECTED_DEFAULTS
    config = NavigationConfig()
    assert config.max_linear_speed == EXPECTED_DEFAULTS["max_linear_speed"]
    assert config.max_angular_speed == EXPECTED_DEFAULTS["max_angular_speed"]
    assert config.heading_stop_threshold == EXPECTED_DEFAULTS[
        "heading_stop_threshold"
    ]
    assert config.goal_tolerance == EXPECTED_DEFAULTS["goal_tolerance"]
    assert config.slowdown_distance == EXPECTED_DEFAULTS["slowdown_distance"]
    assert config.stop_distance == EXPECTED_DEFAULTS["stop_distance"]
    assert config.front_sector_angle == EXPECTED_DEFAULTS["front_sector_angle"]


def test_launch_evaluates_integer_shaped_goals_as_float_and_scan_flag_as_bool():
    description = load_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(
        {"goal_x": "2", "goal_y": "1", "require_scan": "false"}
    )
    for name, declaration in launch_arguments(description).items():
        if name not in {"goal_x", "goal_y", "require_scan"}:
            declaration.visit(context)

    evaluated = evaluate_parameters(context, navigator_parameters(description))[0]

    assert all(isinstance(evaluated[name], float) for name in FLOAT_PARAMETERS)
    assert evaluated["goal_x"] == 2.0
    assert evaluated["goal_y"] == 1.0
    assert evaluated["require_scan"] is False
