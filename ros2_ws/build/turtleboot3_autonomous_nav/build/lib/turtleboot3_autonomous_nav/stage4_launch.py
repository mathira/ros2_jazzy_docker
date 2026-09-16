"""Include the official stage4 description with optional Gazebo GUI."""

import shlex
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchContext, LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node


class Stage4LaunchSource(PythonLaunchDescriptionSource):
    """Gate the GUI-only include in the upstream Jazzy stage4 launcher.

    Upstream has no GUI launch argument. Preserve its world, spawn and bridge
    actions; only wrap the include whose literal gz_args selects GUI-only mode.
    Fail explicitly if upstream changes that contract.
    """

    def _get_launch_description(self, location):
        description = super()._get_launch_description(location)
        actions = []
        gui_count = 0
        for action in description.entities:
            arguments = dict(action.launch_arguments) if isinstance(
                action, IncludeLaunchDescription) else {}
            gz_args = arguments.get('gz_args')
            if isinstance(gz_args, str) and '-g' in shlex.split(gz_args):
                action = GroupAction(
                    actions=[action], condition=IfCondition(LaunchConfiguration('use_gui')))
                gui_count += 1
            actions.append(action)
        if gui_count != 1:
            raise RuntimeError('Official stage4 GUI layout changed; expected one GUI-only include')
        return LaunchDescription(actions)


class InitialRobotBridgeSource(PythonLaunchDescriptionSource):
    """Keep upstream robot bridges when Burger already exists in the world.

    The bridge also carries Gazebo's wheel-derived ``odom -> base_footprint``
    transform, which drifted 1.3 m in a mission and placed the robot and its
    laser away from the map every display draws.  Divert it so the transform
    published from the pose the map itself uses is the only one in the tree.
    """

    def _get_launch_description(self, location):
        description = super()._get_launch_description(location)
        spawners = [action for action in description.entities
                    if isinstance(action, Node) and action.node_executable == 'create']
        if len(spawners) != 1:
            raise RuntimeError('Official Burger spawn layout changed; expected one create process')
        actions = [action for action in description.entities if action is not spawners[0]]
        bridges = [action for action in actions
                   if isinstance(action, Node) and action.node_executable == 'parameter_bridge']
        if not bridges:
            raise RuntimeError('Official Burger spawn layout changed; expected a bridge process')
        diverted = (normalize_to_list_of_substitutions('/tf'),
                    normalize_to_list_of_substitutions('/tf_wheel'))
        for bridge in bridges:
            bridge._Node__remappings = list(bridge._Node__remappings or []) + [diverted]
        return LaunchDescription(actions)


TRUE_ODOMETRY_TOPIC = 'odom_truth'
"""Per-model topic carrying a pose that is not integrated from wheel counts.

Wheel odometry drifted 1.3 m and 163.8 degrees in this arena because the robot
slips on every contact and nothing closes the loop, and the map is drawn with
that pose.  A topic of its own is what makes this usable: bridging the
simulator's list of every model's pose loses the names, so nothing downstream
can tell the robot from an obstacle.
"""

_ROBOT_INCLUDE = (
    '<include><uri>model://turtlebot3_burger</uri><name>burger</name>'
    '<pose>0 0 0.01 0 0 0</pose>'
    '<plugin filename="gz-sim-odometry-publisher-system"'
    ' name="gz::sim::systems::OdometryPublisher">'
    '<odom_frame>odom</odom_frame>'
    '<robot_base_frame>base_footprint</robot_base_frame>'
    f'<odom_topic>{TRUE_ODOMETRY_TOPIC}</odom_topic>'
    '<dimensions>3</dimensions>'
    '</plugin></include>'
)


MAX_CONTACT_CORRECTING_VELOCITY = 1.0
"""Bound on how fast the engine may push interpenetrating bodies apart.

The official world leaves this at 2000 m/s.  Sampling the simulator's own pose
showed what that allows: the robot travelled 1.51 m in one second - fifteen
times its top speed - reached 0.318 m of altitude and landed upside down, half
a metre from anything it could have touched.  It is thrown, not tipped.

A first attempt at 100 m/s changed nothing, which is unsurprising: a robot
separated at even a few metres per second still flies.  This is the scale of
a real contact between a 1 kg robot and a wall.
"""


def _bounded_contact_correction(content: str) -> str:
    """Return the world with its contact correction bounded.

    The upstream world is left untouched; only the derived copy changes, and
    only this one value - the step size and solver stay as ROBOTIS set them.
    """
    world = ET.fromstring(content)
    constraints = world.find('world/physics/ode/constraints')
    if constraints is None:
        raise RuntimeError('Official stage4 physics layout changed; expected ODE constraints')
    correcting = constraints.find('contact_max_correcting_vel')
    if correcting is None:
        raise RuntimeError('Official stage4 physics no longer bounds contact correction')
    correcting.text = f'{MAX_CONTACT_CORRECTING_VELOCITY:.6f}'
    return ET.tostring(world, encoding='unicode')


class TrainingStage4LaunchSource(Stage4LaunchSource):
    """Make the official Burger part of Gazebo's initial reset snapshot.

    Gazebo reset.all deletes entities spawned through /create after startup.
    Derive a temporary world from the untouched upstream stage4 world with its
    standard Burger include, and retain the upstream state publisher/bridges.
    """

    def _get_launch_description(self, location):
        description = super()._get_launch_description(location)
        share = Path(location).parent.parent
        official_world = share / 'worlds' / 'turtlebot3_dqn_stage4.world'
        content = official_world.read_text(encoding='utf-8')
        world = ET.fromstring(content).find('world')
        if world is None or world.get('name') != 'dqn' or content.count('</world>') != 1:
            raise RuntimeError('Official stage4 world layout changed; expected one dqn world')
        temporary = tempfile.TemporaryDirectory(prefix='turtlebot3-training-')
        derived_world = Path(temporary.name) / official_world.name
        derived_world.write_text(
            _bounded_contact_correction(
                content.replace('</world>', f'{_ROBOT_INCLUDE}</world>')),
            encoding='utf-8')
        actions = []
        server_count = spawn_count = 0
        for action in description.entities:
            if isinstance(action, IncludeLaunchDescription):
                arguments = dict(action.launch_arguments)
                gz_args = arguments.get('gz_args')
                if isinstance(gz_args, list) and len(gz_args) == 2 and gz_args[-1] == str(official_world):
                    arguments['gz_args'] = [gz_args[0], str(derived_world)]
                    action = IncludeLaunchDescription(action.launch_description_source,
                                                      launch_arguments=arguments.items())
                    server_count += 1
                elif set(arguments) == {'x_pose', 'y_pose'}:
                    action.launch_description_source.get_launch_description(LaunchContext())
                    if Path(action.launch_description_source.location).name != 'spawn_turtlebot3.launch.py':
                        raise RuntimeError('Official stage4 robot spawn include changed')
                    action = IncludeLaunchDescription(InitialRobotBridgeSource(
                        action.launch_description_source.location),
                        launch_arguments=arguments.items())
                    spawn_count += 1
            actions.append(action)
        if server_count != 1 or spawn_count != 1:
            temporary.cleanup()
            raise RuntimeError('Official stage4 launch layout changed; expected one server and spawn include')

        def cleanup(context):
            temporary.cleanup()

        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
        return LaunchDescription(actions)
