from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_cave_map_metadata_matches_the_stage_world_bounds():
    """Catch a map whose scale or origin no longer matches the 16 m cave."""
    with (PACKAGE_ROOT / 'maps' / 'cave.yaml').open(encoding='utf-8') as stream:
        map_data = yaml.safe_load(stream)

    assert map_data['resolution'] == 0.02
    assert map_data['origin'] == [-8.0, -8.0, 0.0]


def test_nav2_costmaps_use_map_frame_and_stage_laser_scan():
    """Catch Nav2 configurations that cannot consume Stage's scan in map space."""
    with (PACKAGE_ROOT / 'config' / 'nav2_cave.yaml').open(encoding='utf-8') as stream:
        nav_data = yaml.safe_load(stream)

    assert nav_data['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'map'
    assert nav_data['local_costmap']['local_costmap']['ros__parameters']['observation_sources'] == 'scan'
    assert nav_data['local_costmap']['local_costmap']['ros__parameters']['scan']['topic'] == '/base_scan'


def test_nav2_velocity_pipeline_keeps_collision_monitor_between_smoother_and_stage():
    """Catch removal of the final collision-monitor safety stage before Stage cmd_vel."""
    with (PACKAGE_ROOT / 'config' / 'nav2_cave.yaml').open(encoding='utf-8') as stream:
        nav_data = yaml.safe_load(stream)

    collision_monitor = nav_data['collision_monitor']['ros__parameters']
    assert collision_monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert collision_monitor['cmd_vel_out_topic'] == 'cmd_vel'
