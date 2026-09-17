"""Start a clean Stage world without starting the navigation node."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")

    cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "pkill -f '[f]ull.launch.py' || true; "
            "pkill -f '[p]id_navigator' || true; "
            "pkill -x stage || true; pkill -f '[s]tage_ros2' || true",
        ],
        output="screen",
    )

    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("stage_ros2"), "launch", "stage.launch.py"]
            )
        ),
        launch_arguments={"world": world}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="cave"),
            RegisterEventHandler(
                OnProcessExit(target_action=cleanup, on_exit=[stage])
            ),
            cleanup,
        ]
    )
