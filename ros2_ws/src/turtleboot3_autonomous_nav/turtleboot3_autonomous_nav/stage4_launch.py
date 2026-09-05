"""Include the official stage4 description with optional Gazebo GUI."""

import shlex

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
