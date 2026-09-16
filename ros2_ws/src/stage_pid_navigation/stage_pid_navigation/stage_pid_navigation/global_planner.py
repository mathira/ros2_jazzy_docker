"""A* planner over a static Stage occupancy image."""

from heapq import heappop, heappush
import math


def cave_planner(clearance: float = 0.25):
    """Load the installed Stage cave bitmap as an inflated occupancy grid."""
    import cv2
    import numpy as np
    from ament_index_python.packages import get_package_share_directory

    image_path = get_package_share_directory("stage_ros2") + "/world/bitmaps/cave.png"
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Cannot load Stage map: {image_path}")
    pixels_per_meter = image.shape[0] / 16.0
    kernel_size = 2 * math.ceil(clearance * pixels_per_meter) + 1
    wall = (image < 128).astype(np.uint8)
    inflated = cv2.dilate(wall, np.ones((kernel_size, kernel_size), np.uint8)).astype(bool)
    # Image rows grow downward while Stage y grows upward.
    return GridPlanner(np.flipud(inflated), resolution=16.0 / image.shape[0], origin=(-8.0, -8.0))


class GridPlanner:
    def __init__(self, occupied, *, resolution: float, origin: tuple[float, float]):
        self.occupied = occupied
        self.height, self.width = occupied.shape
        self.resolution, self.origin = resolution, origin

    def to_cell(self, point):
        x = min(self.width - 1, max(0, round((point[0] - self.origin[0]) / self.resolution)))
        y = min(self.height - 1, max(0, round((point[1] - self.origin[1]) / self.resolution)))
        return x, y

    def to_point(self, cell):
        return (self.origin[0] + cell[0] * self.resolution, self.origin[1] + cell[1] * self.resolution)

    def plan(self, start, goal):
        start_cell, goal_cell = self.to_cell(start), self.to_cell(goal)
        queue, came_from, costs = [(0.0, start_cell)], {}, {start_cell: 0.0}
        while queue:
            _, current = heappop(queue)
            if current == goal_cell:
                cells = [current]
                while current in came_from:
                    current = came_from[current]
                    cells.append(current)
                cells.reverse()
                points = [self.to_point(cell) for cell in cells]
                points[0], points[-1] = start, goal
                return self._simplify(points)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = current[0] + dx, current[1] + dy
                if not (0 <= nxt[0] < self.width and 0 <= nxt[1] < self.height) or self.occupied[nxt[1], nxt[0]]:
                    continue
                next_cost = costs[current] + math.hypot(dx, dy)
                if next_cost < costs.get(nxt, math.inf):
                    costs[nxt], came_from[nxt] = next_cost, current
                    heappush(queue, (next_cost + math.dist(nxt, goal_cell), nxt))
        raise ValueError("No collision-free path to goal")

    @staticmethod
    def _simplify(points):
        if len(points) < 3:
            return points
        result = [points[0]]
        for previous, current, following in zip(points, points[1:], points[2:]):
            if (current[0] - previous[0]) * (following[1] - current[1]) != (current[1] - previous[1]) * (following[0] - current[0]):
                result.append(current)
        return result + [points[-1]]
