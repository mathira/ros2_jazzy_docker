"""Fixed-size DQN observation construction independent of ROS messages."""

from __future__ import annotations

import math

import numpy as np


LIDAR_SECTOR_COUNT = 12
DIRECTIONAL_GAIN_COUNT = 8
LOCAL_PATCH_SIZE = 8
OBSERVATION_SIZE = (
    LIDAR_SECTOR_COUNT
    + DIRECTIONAL_GAIN_COUNT
    + LOCAL_PATCH_SIZE * LOCAL_PATCH_SIZE
    + 2
)


def build_observation(
    scan_ranges: np.ndarray,
    local_grid: np.ndarray,
    linear_velocity: float,
    angular_velocity: float,
) -> np.ndarray:
    """Build a 86-element vector from LiDAR, local occupancy, and velocity.

    The layout is twelve LiDAR-sector minima, eight directional unknown-area
    gains, an 8 by 8 local occupancy patch, then linear and angular velocity.
    Unknown, free, and occupied patch cells are encoded as ``-1``, ``0``, and
    ``1`` respectively.
    """
    patch = _fixed_local_patch(local_grid)
    return np.concatenate(
        (
            _lidar_sector_minima(scan_ranges),
            _unknown_area_gains(patch),
            _encode_patch(patch).reshape(-1),
            np.asarray(
                [_finite_or_zero(linear_velocity), _finite_or_zero(angular_velocity)],
                dtype=np.float32,
            ),
        )
    ).astype(np.float32, copy=False)


def _lidar_sector_minima(scan_ranges: np.ndarray) -> np.ndarray:
    ranges = np.asarray(scan_ranges, dtype=float).reshape(-1)
    minima = np.zeros(LIDAR_SECTOR_COUNT, dtype=np.float32)
    for index, sector in enumerate(np.array_split(ranges, LIDAR_SECTOR_COUNT)):
        valid = sector[np.isfinite(sector) & (sector > 0.0)]
        if valid.size:
            minima[index] = float(np.min(valid))
    return minima


def _fixed_local_patch(local_grid: np.ndarray) -> np.ndarray:
    grid = np.asarray(local_grid)
    if grid.ndim != 2:
        raise ValueError('local_grid must be two-dimensional')

    patch = np.full((LOCAL_PATCH_SIZE, LOCAL_PATCH_SIZE), -1, dtype=np.int16)
    source_y, source_x = grid.shape
    copy_height = min(source_y, LOCAL_PATCH_SIZE)
    copy_width = min(source_x, LOCAL_PATCH_SIZE)
    source_y_start = max((source_y - copy_height) // 2, 0)
    source_x_start = max((source_x - copy_width) // 2, 0)
    patch_y_start = (LOCAL_PATCH_SIZE - copy_height) // 2
    patch_x_start = (LOCAL_PATCH_SIZE - copy_width) // 2
    patch[
        patch_y_start : patch_y_start + copy_height,
        patch_x_start : patch_x_start + copy_width,
    ] = grid[
        source_y_start : source_y_start + copy_height,
        source_x_start : source_x_start + copy_width,
    ]
    return patch


def _unknown_area_gains(patch: np.ndarray) -> np.ndarray:
    gains = np.zeros(DIRECTIONAL_GAIN_COUNT, dtype=np.float32)
    totals = np.zeros(DIRECTIONAL_GAIN_COUNT, dtype=np.int16)
    center = (LOCAL_PATCH_SIZE - 1) / 2.0
    for row, column in np.ndindex(patch.shape):
        delta_x = column - center
        delta_y = center - row
        if delta_x == 0.0 and delta_y == 0.0:
            continue
        angle = math.atan2(delta_y, delta_x)
        sector = int(math.floor((angle + math.pi / 8.0) / (math.pi / 4.0))) % 8
        totals[sector] += 1
        if patch[row, column] == -1:
            gains[sector] += 1.0
    return np.divide(gains, totals, out=np.zeros_like(gains), where=totals != 0)


def _encode_patch(patch: np.ndarray) -> np.ndarray:
    return np.where(patch == -1, -1.0, np.where(patch >= 50, 1.0, 0.0)).astype(
        np.float32
    )


def _finite_or_zero(value: float) -> float:
    value = float(value)
    return value if np.isfinite(value) else 0.0
