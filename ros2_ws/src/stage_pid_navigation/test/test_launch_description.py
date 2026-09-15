from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


LAUNCH_FILE = (
    Path(__file__).parents[1]
    / "stage_pid_navigation"
    / "launch"
    / "pid_navigation.launch.py"
)


def test_launch_declares_stage_topics_and_goal_arguments():
    spec = spec_from_file_location("pid_navigation_launch", LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    names = {
        action.name
        for action in description.entities
        if hasattr(action, "name")
    }
    expected_arguments = {
        "goal_x",
        "goal_y",
        "odom_topic",
        "scan_topic",
        "cmd_vel_topic",
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
        "require_scan",
    }
    assert expected_arguments <= names

    from launch_ros.actions import Node

    node = next(action for action in description.entities if isinstance(action, Node))
    node_fields = vars(node)
    assert node_fields["_Node__package"] == "stage_pid_navigation"
    assert node_fields["_Node__node_executable"] == "pid_navigator"
    parameters = node_fields["_Node__parameters"][0]
    assert {key[0].text for key in parameters} == expected_arguments

    defaults = {
        action.name: action.default_value
        for action in description.entities
        if hasattr(action, "name") and hasattr(action, "default_value")
    }
    assert defaults["odom_topic"][0].text == "/odom"
    assert defaults["scan_topic"][0].text == "/base_scan"
    assert defaults["cmd_vel_topic"][0].text == "/cmd_vel"
