"""Pure goal-navigation and LiDAR-safety control primitives."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from collections.abc import Iterable
from typing import Optional


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ScanSummary:
    front: float
    left: float
    right: float

    @classmethod
    def clear(cls) -> "ScanSummary":
        return cls(front=math.inf, left=math.inf, right=math.inf)


@dataclass(frozen=True)
class NavigationConfig:
    max_linear_speed: float = 0.35
    max_angular_speed: float = 1.2
    heading_stop_threshold: float = 0.7
    goal_tolerance: float = 0.15
    slowdown_distance: float = 0.9
    stop_distance: float = 0.35
    front_sector_angle: float = 0.7


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float
    reached_goal: bool = False
    blocked: bool = False


class NavigationMode(Enum):
    GO_TO_GOAL = auto()
    FOLLOW_WALL = auto()
    RECOVERY = auto()


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the inclusive range [-pi, pi]."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi and angle > 0.0 else wrapped


def summarize_scan(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    front_sector_angle: float,
) -> ScanSummary:
    """Return minimum valid range in the forward and side scan sectors.

    The front sector is centred on zero radians.  The remainder of the
    left/right hemispheres supplies the side-clearance values used for a
    deterministic obstacle-avoidance turn.
    """
    front = math.inf
    left = math.inf
    right = math.inf
    half_front_sector = abs(front_sector_angle) / 2.0

    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0.0:
            continue
        angle = normalize_angle(angle_min + index * angle_increment)
        if abs(angle) <= half_front_sector:
            front = min(front, distance)
        elif angle > 0.0:
            left = min(left, distance)
        else:
            right = min(right, distance)

    return ScanSummary(front=front, left=left, right=right)


class PidController:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self._integral = 0.0
        self._previous_error: Optional[float] = None

    def update(self, error: float, dt: float) -> float:
        derivative = 0.0
        if dt > 0.0:
            self._integral += error * dt
            self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))
            if self._previous_error is not None:
                derivative = (error - self._previous_error) / dt
            self._previous_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


def compute_command(
    pose: Pose2D,
    goal: tuple[float, float],
    scan: ScanSummary,
    config: NavigationConfig,
    pid: PidController,
    dt: float,
) -> Command:
    """Calculate a goal-seeking velocity command without ROS dependencies."""
    dx = goal[0] - pose.x
    dy = goal[1] - pose.y
    distance = math.hypot(dx, dy)
    if distance <= config.goal_tolerance:
        return Command(linear=0.0, angular=0.0, reached_goal=True)

    heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)
    angular = max(
        -config.max_angular_speed,
        min(config.max_angular_speed, pid.update(heading_error, dt)),
    )
    linear = 0.0 if abs(heading_error) > config.heading_stop_threshold else min(
        config.max_linear_speed, distance
    )
    if math.isfinite(scan.front) and scan.front < config.stop_distance:
        left_clearance = scan.left if math.isfinite(scan.left) else -math.inf
        right_clearance = scan.right if math.isfinite(scan.right) else -math.inf
        turn_left = left_clearance >= right_clearance
        return Command(
            linear=0.0,
            angular=config.max_angular_speed if turn_left else -config.max_angular_speed,
            blocked=True,
        )

    if (
        math.isfinite(scan.front)
        and config.stop_distance < config.slowdown_distance
        and scan.front < config.slowdown_distance
    ):
        linear *= (scan.front - config.stop_distance) / (
            config.slowdown_distance - config.stop_distance
        )
    return Command(linear=linear, angular=angular)


class WallFollower:
    """Reactive Bug-style controller that leaves a wall only on progress."""

    def __init__(self, config: NavigationConfig, pid: PidController, *, goal_standoff: float = 0.55,
                 wall_distance: float = 0.55, wall_follow_linear_speed: float = 0.18,
                 wall_kp: float = 1.5, stuck_timeout: float = 8.0, recovery_turn_duration: float = 1.2):
        self.config, self.pid = config, pid
        self.goal_standoff, self.wall_distance = goal_standoff, wall_distance
        self.wall_follow_linear_speed, self.wall_kp = wall_follow_linear_speed, wall_kp
        self.stuck_timeout, self.recovery_turn_duration = stuck_timeout, recovery_turn_duration
        self.mode = NavigationMode.GO_TO_GOAL
        self.follow_left = True
        self._hit_distance = math.inf
        self._best_distance = math.inf
        self._last_progress_time = 0.0
        self._recovery_start = 0.0

    def update(self, pose: Pose2D, goal: tuple[float, float], scan: ScanSummary, now: float) -> Command:
        distance = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
        if distance <= self.goal_standoff:
            return Command(0.0, 0.0, reached_goal=True)
        if self.mode is NavigationMode.GO_TO_GOAL:
            command = compute_command(pose, goal, scan, self.config, self.pid, 0.1)
            if scan.front < self.config.stop_distance:
                self.follow_left = scan.left >= scan.right
                self.mode = NavigationMode.FOLLOW_WALL
                self._hit_distance = self._best_distance = distance
                self._last_progress_time = now
            return command
        if self.mode is NavigationMode.RECOVERY:
            if now - self._recovery_start >= self.recovery_turn_duration:
                self.mode = NavigationMode.FOLLOW_WALL
                self._last_progress_time = now
            turn = self.config.max_angular_speed if self.follow_left else -self.config.max_angular_speed
            return Command(0.0, turn, blocked=True)
        # FOLLOW_WALL
        if distance < self._best_distance - 0.05:
            self._best_distance, self._last_progress_time = distance, now
        heading = normalize_angle(math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw)
        if scan.front >= self.config.slowdown_distance and distance < self._hit_distance - 0.1 and abs(heading) < 0.5:
            self.mode = NavigationMode.GO_TO_GOAL
            return compute_command(pose, goal, scan, self.config, self.pid, 0.1)
        if now - self._last_progress_time >= self.stuck_timeout:
            self.mode = NavigationMode.RECOVERY
            self.follow_left = not self.follow_left
            self._recovery_start = now
            return self.update(pose, goal, scan, now)
        side = scan.left if self.follow_left else scan.right
        if scan.front < self.config.stop_distance:
            angular = -self.config.max_angular_speed if self.follow_left else self.config.max_angular_speed
            return Command(0.0, angular, blocked=True)
        side_error = (self.wall_distance - side) if self.follow_left else (side - self.wall_distance)
        angular = max(-self.config.max_angular_speed, min(self.config.max_angular_speed, self.wall_kp * side_error))
        return Command(self.wall_follow_linear_speed, angular)
