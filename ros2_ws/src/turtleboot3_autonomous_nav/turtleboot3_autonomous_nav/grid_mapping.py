"""Pure occupancy-grid mapping primitives for odometry and LiDAR data."""

from __future__ import annotations

from typing import TYPE_CHECKING, Iterator

import numpy as np

if TYPE_CHECKING:
    from nav_msgs.msg import OccupancyGrid


class OccupancyGridModel:
    """Accumulate bounded LiDAR ray observations in an odometric grid.

    Grid cells use the standard occupancy values: ``-1`` for unknown, ``0``
    for free, and ``100`` for occupied.  Coverage counts cells that have ever
    become known, so it cannot decrease when later evidence changes a cell.
    """

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin: tuple[float, float],
        occupied_threshold: int = 1,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError('width and height must be positive')
        if resolution <= 0.0:
            raise ValueError('resolution must be positive')
        if occupied_threshold <= 0:
            raise ValueError('occupied_threshold must be positive')

        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.occupied_threshold = int(occupied_threshold)

        self._grid = np.full((self.height, self.width), self.UNKNOWN, dtype=np.int8)
        self._free_evidence = np.zeros((self.height, self.width), dtype=np.uint16)
        self._occupied_evidence = np.zeros(
            (self.height, self.width), dtype=np.uint16
        )
        self._known_cells = 0

    def update_scan(
        self,
        pose: tuple[float, float, float],
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        range_max: float,
    ) -> int:
        """Apply a scan and return the number of cells newly made known.

        Non-finite and non-positive readings are ignored.  Finite readings at
        or beyond ``range_max`` trace free space only, while shorter readings
        also contribute occupied evidence at their endpoint.
        """
        pose_x, pose_y, pose_yaw = pose
        measurements_are_finite = np.all(
            np.isfinite(
                (pose_x, pose_y, pose_yaw, angle_min, angle_increment, range_max)
            )
        )
        if not measurements_are_finite or range_max <= 0.0:
            return 0

        start = self._world_to_cell(pose_x, pose_y)
        newly_known = 0
        flattened_ranges = np.asarray(ranges, dtype=float).reshape(-1)

        for index, measured_range in enumerate(flattened_ranges):
            if not np.isfinite(measured_range) or measured_range <= 0.0:
                continue

            is_hit = measured_range < range_max
            ray_length = min(float(measured_range), float(range_max))
            angle = pose_yaw + angle_min + index * angle_increment
            endpoint = self._world_to_cell(
                pose_x + ray_length * np.cos(angle),
                pose_y + ray_length * np.sin(angle),
            )
            cells = list(self._bresenham_cells(*start, *endpoint))
            if not cells:
                continue

            free_cells = cells[1:]
            if is_hit:
                free_cells = free_cells[:-1]

            for cell_x, cell_y in free_cells:
                newly_known += self._apply_free(cell_x, cell_y)

            if is_hit:
                end_x, end_y = cells[-1]
                newly_known += self._apply_occupied(end_x, end_y)

        return newly_known

    def value_at(self, x: float, y: float) -> int:
        """Return the occupancy value at world coordinates, or unknown out of bounds."""
        cell_x, cell_y = self._world_to_cell(x, y)
        if not self._in_bounds(cell_x, cell_y):
            return self.UNKNOWN
        return int(self._grid[cell_y, cell_x])

    def coverage_fraction(self) -> float:
        """Return the fraction of cells that have changed from unknown."""
        return self._known_cells / float(self.width * self.height)

    def to_message(self, stamp: object, frame_id: str = 'odom') -> 'OccupancyGrid':
        """Create a ROS ``OccupancyGrid`` message without coupling the core to ROS."""
        try:
            from nav_msgs.msg import OccupancyGrid
        except ImportError as error:  # pragma: no cover - exercised in ROS only
            raise RuntimeError('nav_msgs is required to create an OccupancyGrid') from error

        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.info.resolution = self.resolution
        message.info.width = self.width
        message.info.height = self.height
        message.info.origin.position.x = self.origin[0]
        message.info.origin.position.y = self.origin[1]
        message.info.origin.orientation.w = 1.0
        message.data = self._grid.reshape(-1).astype(int).tolist()
        return message

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(np.floor((x - self.origin[0]) / self.resolution)),
            int(np.floor((y - self.origin[1]) / self.resolution)),
        )

    def _apply_free(self, cell_x: int, cell_y: int) -> int:
        if not self._in_bounds(cell_x, cell_y):
            return 0
        self._free_evidence[cell_y, cell_x] += 1
        return self._update_cell_value(cell_x, cell_y)

    def _apply_occupied(self, cell_x: int, cell_y: int) -> int:
        if not self._in_bounds(cell_x, cell_y):
            return 0
        self._occupied_evidence[cell_y, cell_x] += 1
        return self._update_cell_value(cell_x, cell_y)

    def _update_cell_value(self, cell_x: int, cell_y: int) -> int:
        old_value = self._grid[cell_y, cell_x]
        free_evidence = self._free_evidence[cell_y, cell_x]
        occupied_evidence = self._occupied_evidence[cell_y, cell_x]

        if (
            occupied_evidence >= self.occupied_threshold
            and occupied_evidence >= free_evidence
        ):
            self._grid[cell_y, cell_x] = self.OCCUPIED
        elif free_evidence > 0:
            self._grid[cell_y, cell_x] = self.FREE

        if old_value == self.UNKNOWN and self._grid[cell_y, cell_x] != self.UNKNOWN:
            self._known_cells += 1
            return 1
        return 0

    def _in_bounds(self, cell_x: int, cell_y: int) -> bool:
        return 0 <= cell_x < self.width and 0 <= cell_y < self.height

    @staticmethod
    def _bresenham_cells(
        start_x: int, start_y: int, end_x: int, end_y: int
    ) -> Iterator[tuple[int, int]]:
        """Yield the integral cells on a ray, including both endpoints."""
        delta_x = abs(end_x - start_x)
        step_x = 1 if start_x < end_x else -1
        delta_y = -abs(end_y - start_y)
        step_y = 1 if start_y < end_y else -1
        error = delta_x + delta_y

        while True:
            yield start_x, start_y
            if start_x == end_x and start_y == end_y:
                return
            twice_error = 2 * error
            if twice_error >= delta_y:
                error += delta_y
                start_x += step_x
            if twice_error <= delta_x:
                error += delta_x
                start_y += step_y
