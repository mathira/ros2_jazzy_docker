import math

from stage_pid_navigation.control import (
    NavigationConfig,
    NavigationMode,
    PidController,
    Pose2D,
    ScanSummary,
    WallFollower,
    compute_command,
    normalize_angle,
    summarize_scan,
)


def test_wall_follower_enters_wall_following_and_selects_clearer_side():
    follower = WallFollower(make_config(), PidController(1.8, 0.0, 0.15, 1.0))
    command = follower.update(
        Pose2D(0.0, 0.0, 0.0), (5.0, 4.0), ScanSummary(0.2, 1.0, 0.4), 0.0
    )
    assert follower.mode is NavigationMode.FOLLOW_WALL
    assert follower.follow_left is True
    assert command.linear == 0.0 and command.angular > 0.0


def test_wall_follower_stops_while_turning_when_front_is_blocked():
    follower = WallFollower(make_config(), PidController(1.8, 0.0, 0.15, 1.0))
    follower.mode = NavigationMode.FOLLOW_WALL
    follower.follow_left = True
    command = follower.update(
        Pose2D(1.2, -0.5, 0.0), (4.2, 4.0), ScanSummary(0.20, 0.20, 1.76), 1.0
    )
    assert command.linear == 0.0
    assert command.angular < 0.0


def test_wall_follower_returns_to_goal_when_clear_and_closer():
    follower = WallFollower(make_config(), PidController(1.8, 0.0, 0.15, 1.0))
    follower.update(Pose2D(0.0, 0.0, 0.0), (5.0, 0.0), ScanSummary(0.2, 1.0, 0.4), 0.0)
    follower.update(Pose2D(1.0, 0.0, 0.0), (5.0, 0.0), ScanSummary.clear(), 1.0)
    assert follower.mode is NavigationMode.GO_TO_GOAL


def test_wall_follower_reaches_safe_standoff_before_block_center():
    follower = WallFollower(make_config(), PidController(1.8, 0.0, 0.15, 1.0), goal_standoff=0.55)
    command = follower.update(Pose2D(4.46, 4.0, 0.0), (5.0, 4.0), ScanSummary.clear(), 0.0)
    assert command.reached_goal


def make_config() -> NavigationConfig:
    return NavigationConfig(
        max_linear_speed=0.3,
        max_angular_speed=1.0,
        heading_stop_threshold=0.35,
        goal_tolerance=0.15,
        slowdown_distance=0.75,
        stop_distance=0.25,
    )


def test_normalize_angle_wraps_across_pi():
    assert normalize_angle(3.5) < 0.0


def test_pid_clamps_integral_accumulation():
    pid = PidController(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5)

    assert pid.update(error=2.0, dt=1.0) == 0.5


def test_controller_stops_inside_goal_tolerance():
    command = compute_command(
        Pose2D(1.0, 1.0, 0.0),
        (1.05, 1.0),
        ScanSummary.clear(),
        make_config(),
        PidController(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0),
        0.1,
    )

    assert command.reached_goal
    assert command.linear == 0.0
    assert command.angular == 0.0


def test_controller_does_not_advance_until_heading_is_aligned():
    command = compute_command(
        Pose2D(0.0, 0.0, 3.14),
        (1.0, 0.0),
        ScanSummary.clear(),
        make_config(),
        PidController(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0),
        0.1,
    )

    assert command.linear == 0.0
    assert command.angular < 0.0


def test_obstacle_inside_stop_distance_blocks_forward_motion_and_turns_to_clearer_side():
    command = compute_command(
        Pose2D(0.0, 0.0, 0.0),
        (2.0, 0.0),
        ScanSummary(front=0.2, left=0.9, right=0.4),
        make_config(),
        PidController(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0),
        0.1,
    )

    assert command.blocked
    assert command.linear == 0.0
    assert command.angular > 0.0


def test_obstacle_inside_slowdown_distance_reduces_linear_speed():
    config = make_config()
    clear = compute_command(
        Pose2D(0.0, 0.0, 0.0),
        (2.0, 0.0),
        ScanSummary.clear(),
        config,
        PidController(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0),
        0.1,
    )
    slowed = compute_command(
        Pose2D(0.0, 0.0, 0.0),
        (2.0, 0.0),
        ScanSummary(front=0.6, left=1.0, right=1.0),
        config,
        PidController(kp=1.0, ki=0.0, kd=0.0, integral_limit=1.0),
        0.1,
    )

    assert 0.0 < slowed.linear < clear.linear


def test_scan_summary_ignores_nan_infinity_and_ranges_outside_front_sector():
    ranges = [math.nan, math.inf, 0.8, 0.4]

    summary = summarize_scan(
        ranges=ranges,
        angle_min=-0.2,
        angle_increment=0.2,
        front_sector_angle=0.5,
    )

    assert summary.front == 0.8
