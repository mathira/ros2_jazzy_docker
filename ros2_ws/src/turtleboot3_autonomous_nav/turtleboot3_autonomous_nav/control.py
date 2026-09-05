"""Pure action-to-velocity safety decisions."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


FORWARD = 0
SOFT_LEFT = 1
SOFT_RIGHT = 2
LEFT = 3
RIGHT = 4
RECOVER = 5


@dataclass(frozen=True)
class ControlConfig:
    """Limits and emergency-stop threshold for differential-drive commands."""

    stop_distance: float = 0.20
    linear_speed: float = 0.15
    soft_turn_speed: float = 0.5
    turn_speed: float = 1.0
    emergency_turn_speed: float = 1.0
    recovery_turn_speed: float = 1.2
    max_linear_speed: float = 0.22
    max_angular_speed: float = 1.5


@dataclass(frozen=True)
class TwistDecision:
    """A bounded command plus the safety and recovery events it caused."""

    linear_x: float
    angular_z: float
    intervention: bool
    recovery: bool = False


def stale_twist() -> TwistDecision:
    """Return the fail-safe decision used when required sensor input is stale."""
    return TwistDecision(0.0, 0.0, intervention=True, recovery=False)


def control_sector_ranges(
    scan_ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_max: float,
) -> np.ndarray:
    """Reduce a scan to ``[front, left, right]`` safety-sector minima.

    A sector with no positive finite reading is represented by ``0.0`` rather
    than assumed clear. A positive infinite return represents a valid
    out-of-range reading only when the scan provides a valid ``range_max``.
    """
    ranges = np.asarray(scan_ranges, dtype=float).reshape(-1)
    if ranges.size == 0 or not np.isfinite(angle_increment) or angle_increment == 0.0:
        return np.zeros(3, dtype=float)

    angles = angle_min + np.arange(ranges.size) * angle_increment
    wrapped_angles = _wrap_angles(angles)
    masks = (
        np.abs(wrapped_angles) <= math.pi / 6.0,
        (wrapped_angles > math.pi / 6.0)
        & (wrapped_angles <= 5.0 * math.pi / 6.0),
        (wrapped_angles < -math.pi / 6.0)
        & (wrapped_angles >= -5.0 * math.pi / 6.0),
    )
    maximum = float(range_max)
    has_valid_maximum = np.isfinite(maximum) and maximum > 0.0
    sectors = np.zeros(3, dtype=float)
    for index, mask in enumerate(masks):
        sector = ranges[mask]
        finite_positive = sector[np.isfinite(sector) & (sector > 0.0)]
        if finite_positive.size:
            sectors[index] = float(np.min(finite_positive))
        elif has_valid_maximum and np.any(np.isposinf(sector)):
            sectors[index] = maximum
    return sectors


def safe_twist(
    action: int,
    sector_ranges: np.ndarray,
    stalled: bool,
    config: ControlConfig,
) -> TwistDecision:
    """Translate an action into a safe twist using ``[front, left, right]`` ranges."""
    ranges = np.asarray(sector_ranges, dtype=float).reshape(-1)
    front_range = ranges[0] if ranges.size else 0.0
    front_is_blocked = not np.isfinite(front_range) or front_range <= config.stop_distance

    if stalled or action == RECOVER:
        return TwistDecision(
            0.0,
            _clearer_side_turn(ranges, config.recovery_turn_speed, config),
            intervention=False,
            recovery=True,
        )

    commanded = _action_twist(action, config)
    if commanded is None:
        return TwistDecision(0.0, 0.0, intervention=True)

    linear_x, angular_z = commanded
    if linear_x > 0.0 and front_is_blocked:
        return TwistDecision(
            0.0,
            _clearer_side_turn(ranges, config.emergency_turn_speed, config),
            intervention=True,
        )

    return TwistDecision(linear_x, angular_z, intervention=False)


def _action_twist(action: int, config: ControlConfig) -> tuple[float, float] | None:
    """Return the bounded nominal twist for a valid discrete action."""
    linear_speed = _clamp_positive(config.linear_speed, config.max_linear_speed)
    soft_turn = _clamp_signed(config.soft_turn_speed, config.max_angular_speed)
    turn = _clamp_signed(config.turn_speed, config.max_angular_speed)
    actions = {
        FORWARD: (linear_speed, 0.0),
        SOFT_LEFT: (linear_speed, soft_turn),
        SOFT_RIGHT: (linear_speed, -soft_turn),
        LEFT: (0.0, turn),
        RIGHT: (0.0, -turn),
    }
    return actions.get(int(action))


def _clamp_positive(value: float, maximum: float) -> float:
    return min(max(float(value), 0.0), max(float(maximum), 0.0))


def _clamp_signed(value: float, maximum: float) -> float:
    limit = max(float(maximum), 0.0)
    return min(max(float(value), -limit), limit)


def _clearer_side_turn(
    ranges: np.ndarray, turn_speed: float, config: ControlConfig
) -> float:
    """Return a bounded turn towards the largest finite side clearance."""
    left_range = ranges[1] if ranges.size > 1 else 0.0
    right_range = ranges[2] if ranges.size > 2 else 0.0
    left_clearance = float(left_range) if np.isfinite(left_range) else 0.0
    right_clearance = float(right_range) if np.isfinite(right_range) else 0.0
    direction = 1.0 if left_clearance >= right_clearance else -1.0
    magnitude = _clamp_positive(turn_speed, config.max_angular_speed)
    return direction * magnitude


def _wrap_angles(angles: np.ndarray) -> np.ndarray:
    return (angles + math.pi) % (2.0 * math.pi) - math.pi
