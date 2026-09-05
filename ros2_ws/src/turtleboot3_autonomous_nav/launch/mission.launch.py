"""Autonomous exploration in the official TurtleBot3 stage4 world."""

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from turtleboot3_autonomous_nav.stage4_launch import Stage4LaunchSource


def generate_launch_description():
    package = 'turtleboot3_autonomous_nav'
    share = FindPackageShare(package)
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('model_path', description='Trusted trained checkpoint path'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', PathJoinSubstitution([
            FindPackagePrefix('turtlebot3_gazebo'), 'lib', 'turtlebot3_gazebo'])),
        GroupAction(actions=[
            SetRemap(src='/cmd_vel', dst='/official_cmd_vel_stamped'),
            IncludeLaunchDescription(Stage4LaunchSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 'launch', 'turtlebot3_dqn_stage4.launch.py'])),
                launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        ]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='exploration_velocity_bridge',
             arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'], parameters=[sim]),
        Node(package=package, executable='coverage_mapper', name='coverage_mapper',
             parameters=[PathJoinSubstitution([share, 'config', 'exploration.yaml']), sim]),
        Node(package=package, executable='observation_builder', name='observation_builder', parameters=[sim]),
        Node(package=package, executable='safe_motion_controller', name='safe_motion_controller', parameters=[sim]),
        Node(package=package, executable='dqn_explorer', name='dqn_explorer', parameters=[sim, {
            'model_path': ParameterValue(LaunchConfiguration('model_path'), value_type=str)}]),
        Node(package=package, executable='exploration_visualizer', name='exploration_visualizer',
             parameters=[sim], condition=IfCondition(LaunchConfiguration('use_rviz'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', parameters=[sim],
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'exploration.rviz'])],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
