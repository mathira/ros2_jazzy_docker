from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENT_DEFAULTS = {
    "goal_x": "0.0",
    "goal_y": "0.0",
    "odom_topic": "/odom",
    "scan_topic": "/base_scan",
    "cmd_vel_topic": "/cmd_vel",
    "control_rate": "10.0",
    "kp": "1.0",
    "ki": "0.0",
    "kd": "0.0",
    "integral_limit": "1.0",
    "max_linear_speed": "0.3",
    "max_angular_speed": "1.0",
    "heading_stop_threshold": "0.35",
    "goal_tolerance": "0.15",
    "slowdown_distance": "0.75",
    "stop_distance": "0.25",
    "front_sector_angle": "0.5",
    "require_scan": "true",
}


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in ARGUMENT_DEFAULTS.items()
    ]
    navigator = Node(
        package="stage_pid_navigation",
        executable="pid_navigator",
        name="pid_navigator",
        output="screen",
        parameters=[
            {
                name: LaunchConfiguration(name)
                for name in ARGUMENT_DEFAULTS
            }
        ],
    )
    return LaunchDescription([*declarations, navigator])
