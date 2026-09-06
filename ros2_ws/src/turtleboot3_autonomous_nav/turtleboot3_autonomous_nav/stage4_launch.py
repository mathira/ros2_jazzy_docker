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
    """Keep upstream robot bridges when Burger already exists in the world."""

    def _get_launch_description(self, location):
        description = super()._get_launch_description(location)
        spawners = [action for action in description.entities
                    if isinstance(action, Node) and action.node_executable == 'create']
        if len(spawners) != 1:
            raise RuntimeError('Official Burger spawn layout changed; expected one create process')
        return LaunchDescription([action for action in description.entities
                                  if action is not spawners[0]])


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
        derived_world.write_text(content.replace('</world>',
            '<include><uri>model://turtlebot3_burger</uri><name>burger</name>'
            '<pose>0 0 0.01 0 0 0</pose></include></world>'), encoding='utf-8')
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
