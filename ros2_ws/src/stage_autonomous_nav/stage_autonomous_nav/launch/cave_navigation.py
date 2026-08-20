"""Launch Stage, ground-truth localization, Nav2, and RViz for the cave."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the single-robot Stage cave navigation launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    ground_truth_topic = LaunchConfiguration('ground_truth_topic')

    package_share = FindPackageShare('stage_autonomous_nav')
    map_yaml = PathJoinSubstitution([package_share, 'maps', 'cave.yaml'])
    nav2_params = PathJoinSubstitution([package_share, 'config', 'nav2_cave.yaml'])
    rviz_config = PathJoinSubstitution(
        [package_share, 'rviz', 'cave_navigation.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='cave'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('scan_topic', default_value='/base_scan'),
        DeclareLaunchArgument('ground_truth_topic', default_value='/ground_truth'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('stage_ros2'), 'launch', 'stage.launch.py',
            ])),
            launch_arguments={
                'world': world,
                'enforce_prefixes': 'false',
                'one_tf_tree': 'true',
                'use_stamped_velocity': 'false',
                'use_ackermann': 'false',
            }.items(),
        ),
        Node(
            package='stage_autonomous_nav',
            executable='ground_truth_localizer',
            name='ground_truth_localizer',
            parameters=[{
                'use_sim_time': use_sim_time,
                'ground_truth_topic': ground_truth_topic,
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py',
            ])),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_composition': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
