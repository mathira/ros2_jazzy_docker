from stage_autonomous_nav.launch.cave_navigation import generate_launch_description


def test_cave_navigation_declares_supported_arguments():
    names = {
        action.name
        for action in generate_launch_description().entities
        if hasattr(action, 'name')
    }

    assert {'world', 'map_frame', 'odom_frame', 'base_frame', 'scan_topic', 'rviz'} <= names
