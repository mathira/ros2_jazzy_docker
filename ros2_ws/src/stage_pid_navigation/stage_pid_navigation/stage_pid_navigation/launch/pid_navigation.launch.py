from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENT_DEFAULTS = {
    "goal_x": "5.0",
    "goal_y": "4.0",
    "odom_topic": "/ground_truth",
    "scan_topic": "/base_scan",
    "cmd_vel_topic": "/cmd_vel",
    "control_rate": "10.0",
    "kp": "1.8",
    "ki": "0.0",
    "kd": "0.15",
    "integral_limit": "1.0",
    "max_linear_speed": "0.35",
    "max_angular_speed": "1.2",
    "heading_stop_threshold": "0.7",
    "goal_tolerance": "0.15",
    "slowdown_distance": "0.9",
    "stop_distance": "0.35",
    "front_sector_angle": "0.7",
    "require_scan": "true",
    "goal_standoff": "0.65",
    "wall_distance": "0.55",
    "wall_follow_linear_speed": "0.18",
    "wall_kp": "1.5",
    "stuck_timeout": "8.0",
    "recovery_turn_duration": "1.2",
    "use_global_planner": "true",
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
    "goal_standoff",
    "wall_distance",
    "wall_follow_linear_speed",
    "wall_kp",
    "stuck_timeout",
    "recovery_turn_duration",
}


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in ARGUMENT_DEFAULTS.items()
    ]
    parameter_names = tuple(ARGUMENT_DEFAULTS)
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        if name in FLOAT_PARAMETERS
        else ParameterValue(LaunchConfiguration(name), value_type=bool)
        if name in {"require_scan", "use_global_planner"}
        else LaunchConfiguration(name)
        for name in parameter_names
    }
    navigator = Node(
        package="stage_pid_navigation",
        executable="pid_navigator",
        name="pid_navigator",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([*declarations, navigator])
