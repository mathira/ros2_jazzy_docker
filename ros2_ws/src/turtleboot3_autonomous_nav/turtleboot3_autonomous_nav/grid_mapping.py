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
            raise ValueError("width and height must be positive")
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if occupied_threshold <= 0:
            raise ValueError("occupied_threshold must be positive")

        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.occupied_threshold = int(occupied_threshold)

        self._grid = np.full((self.height, self.width), self.UNKNOWN, dtype=np.int8)
        self._free_evidence = np.zeros((self.height, self.width), dtype=np.uint16)
        self._occupied_evidence = np.zeros((self.height, self.width), dtype=np.uint16)
        self._known_cells = 0

    def reset(self) -> None:
        """Clear every observation so the next episode starts at zero coverage."""
        self._grid.fill(self.UNKNOWN)
        self._free_evidence.fill(0)
        self._occupied_evidence.fill(0)
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

        newly_known = 0
        flattened_ranges = np.asarray(ranges, dtype=float).reshape(-1)
        robot_is_in_map = self._point_in_map(pose_x, pose_y)

        for index, measured_range in enumerate(flattened_ranges):
            if not np.isfinite(measured_range) or measured_range <= 0.0:
                continue

            is_hit = measured_range < range_max
            ray_length = min(float(measured_range), float(range_max))
            angle = pose_yaw + angle_min + index * angle_increment
            direction_x = float(np.cos(angle))
            direction_y = float(np.sin(angle))
            clipped_ray = self._clip_ray_to_map(
                pose_x, pose_y, direction_x, direction_y, ray_length
            )
            if clipped_ray is None:
                continue

            clipped_start, clipped_end = clipped_ray
            start = self._bounded_world_to_cell(*clipped_start)
            endpoint = self._bounded_world_to_cell(*clipped_end)
            cells = list(self._bresenham_cells(*start, *endpoint))

            free_cells = cells[1:] if robot_is_in_map else cells
            endpoint_is_in_map = is_hit and self._point_in_map(
                pose_x + ray_length * direction_x,
                pose_y + ray_length * direction_y,
            )
            if endpoint_is_in_map:
                free_cells = free_cells[:-1]

            for cell_x, cell_y in free_cells:
                newly_known += self._apply_free(cell_x, cell_y)

            if endpoint_is_in_map:
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

    def to_message(self, stamp: object, frame_id: str = "odom") -> "OccupancyGrid":
        """Create a ROS ``OccupancyGrid`` message without coupling the core to ROS."""
        try:
            from nav_msgs.msg import OccupancyGrid
        except ImportError as error:  # pragma: no cover - exercised in ROS only
            raise RuntimeError(
                "nav_msgs is required to create an OccupancyGrid"
            ) from error

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

    def _bounded_world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """Convert a clipped point to a valid cell, including the upper edge."""
        cell_x, cell_y = self._world_to_cell(x, y)
        return (
            min(max(cell_x, 0), self.width - 1),
            min(max(cell_y, 0), self.height - 1),
        )

    def _point_in_map(self, x: float, y: float) -> bool:
        min_x, min_y = self.origin
        return (
            min_x <= x < min_x + self.width * self.resolution
            and min_y <= y < min_y + self.height * self.resolution
        )

    def _clip_ray_to_map(
        self,
        start_x: float,
        start_y: float,
        direction_x: float,
        direction_y: float,
        length: float,
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        """Clip a ray segment to the map before discrete traversal.

        Distances are clipped parametrically, avoiding an intermediate endpoint
        or Bresenham path whose size depends on an out-of-map sensor range.
        """
        min_x, min_y = self.origin
        max_x = min_x + self.width * self.resolution
        max_y = min_y + self.height * self.resolution
        enter_distance = 0.0
        exit_distance = length

        for position, direction, lower, upper in (
            (start_x, direction_x, min_x, max_x),
            (start_y, direction_y, min_y, max_y),
        ):
            if direction == 0.0:
                if position < lower or position >= upper:
                    return None
                continue

            lower_distance = (lower - position) / direction
            upper_distance = (upper - position) / direction
            enter_distance = max(enter_distance, min(lower_distance, upper_distance))
            exit_distance = min(exit_distance, max(lower_distance, upper_distance))
            if enter_distance > exit_distance:
                return None

        return (
            (
                start_x + enter_distance * direction_x,
                start_y + enter_distance * direction_y,
            ),
            (
                start_x + exit_distance * direction_x,
                start_y + exit_distance * direction_y,
            ),
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
