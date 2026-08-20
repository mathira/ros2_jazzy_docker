from math import isclose, pi

from stage_autonomous_nav.transform_math import map_to_odom


def test_map_to_odom_is_identity_for_equal_base_poses():
    assert map_to_odom(2.0, -1.0, 0.4, 2.0, -1.0, 0.4) == (0.0, 0.0, 0.0)


def test_map_to_odom_composes_global_pose_with_inverse_odom_pose():
    x, y, yaw = map_to_odom(10.0, 5.0, pi / 2.0, 1.0, 0.0, 0.0)
    assert isclose(x, 10.0, abs_tol=1e-9)
    assert isclose(y, 4.0, abs_tol=1e-9)
    assert isclose(yaw, pi / 2.0, abs_tol=1e-9)
