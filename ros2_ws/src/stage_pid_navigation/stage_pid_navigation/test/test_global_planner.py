import math

import numpy as np

from stage_pid_navigation.global_planner import GridPlanner


def test_planner_routes_through_open_corridor_around_wall():
    occupied = np.zeros((20, 20), dtype=bool)
    occupied[5:15, 10] = True
    planner = GridPlanner(occupied, resolution=1.0, origin=(-10.0, -10.0))

    path = planner.plan((-5.0, 0.0), (5.0, 0.0))

    assert path[0] == (-5.0, 0.0)
    assert path[-1] == (5.0, 0.0)
    assert any(abs(y) >= 5.0 for _, y in path)


def test_planner_never_returns_an_occupied_cell():
    occupied = np.zeros((10, 10), dtype=bool)
    occupied[4:6, 4:6] = True
    planner = GridPlanner(occupied, resolution=1.0, origin=(0.0, 0.0))

    path = planner.plan((1.0, 1.0), (8.0, 8.0))

    assert all(not occupied[planner.to_cell(point)[1], planner.to_cell(point)[0]] for point in path)


def test_planner_finds_closest_reachable_cell_when_goal_is_blocked():
    occupied = np.zeros((10, 10), dtype=bool)
    occupied[4:6, 4:6] = True
    planner = GridPlanner(occupied, resolution=1.0, origin=(0.0, 0.0))

    path = planner.plan_to_closest_reachable((1.0, 1.0), (4.5, 4.5))

    assert path[0] == (1.0, 1.0)
    assert not planner.occupied[planner.to_cell(path[-1])[1], planner.to_cell(path[-1])[0]]
    assert math.dist(path[-1], (4.5, 4.5)) < 2.0
