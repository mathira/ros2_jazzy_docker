from math import isclose, pi, sqrt

from stage_autonomous_nav.ground_truth_localizer import (
    quaternion_from_yaw,
    yaw_from_quaternion,
)
from stage_autonomous_nav.transform_math import map_to_odom


def test_map_to_odom_is_identity_for_equal_base_poses():
    assert map_to_odom(2.0, -1.0, 0.4, 2.0, -1.0, 0.4) == (0.0, 0.0, 0.0)


def test_map_to_odom_composes_global_pose_with_inverse_odom_pose():
    x, y, yaw = map_to_odom(10.0, 5.0, pi / 2.0, 1.0, 0.0, 0.0)
    assert isclose(x, 10.0, abs_tol=1e-9)
    assert isclose(y, 4.0, abs_tol=1e-9)
    assert isclose(yaw, pi / 2.0, abs_tol=1e-9)


def test_quaternion_from_yaw_converts_pi_over_two_to_planar_quaternion():
    _, _, z, w = quaternion_from_yaw(pi / 2.0)

    assert isclose(z, sqrt(0.5), abs_tol=1e-9)
    assert isclose(w, sqrt(0.5), abs_tol=1e-9)


def test_yaw_from_quaternion_converts_planar_quaternion_to_pi_over_two():
    yaw = yaw_from_quaternion(0.0, 0.0, sqrt(0.5), sqrt(0.5))

    assert isclose(yaw, pi / 2.0, abs_tol=1e-9)
