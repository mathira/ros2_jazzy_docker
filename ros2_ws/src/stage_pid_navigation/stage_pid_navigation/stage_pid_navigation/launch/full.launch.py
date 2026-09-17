"""Start a clean Stage world and the navigation node together."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    static_stage = LaunchConfiguration("static_stage")
    world = LaunchConfiguration("world")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_standoff = LaunchConfiguration("goal_standoff")

    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("stage_ros2"), "launch", "stage.launch.py"]
            )
        ),
        launch_arguments={"world": world}.items(),
        condition=UnlessCondition(static_stage),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("stage_pid_navigation"), "launch", "pid_navigation.launch.py"]
            )
        ),
        launch_arguments={
            "goal_x": goal_x,
            "goal_y": goal_y,
            "goal_standoff": goal_standoff,
        }.items(),
    )

    cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "pkill -f '[s]tage_world.launch.py' || true; "
            "pkill -f '[p]id_navigator' || true; "
            "if [ \"$1\" = \"false\" ]; then "
            "pkill -x stage || true; pkill -f '[s]tage_ros2' || true; fi",
            "cleanup",
            static_stage,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="cave"),
            DeclareLaunchArgument("goal_x", default_value="5.0"),
            DeclareLaunchArgument("goal_y", default_value="4.0"),
            DeclareLaunchArgument("goal_standoff", default_value="0.65"),
            DeclareLaunchArgument(
                "static_stage",
                default_value="false",
                description="Reuse the existing Stage process instead of starting a new world.",
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=cleanup, on_exit=[stage_launch, navigation_launch])
            ),
            cleanup,
        ]
    )
