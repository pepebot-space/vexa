"""
Occupancy Grid Map for SLAM
Based on: RELAX paper (https://arxiv.org/html/2309.08095v2)

Creates 2D occupancy grid map from LiDAR scans for autonomous navigation.
"""

import numpy as np
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class OccupancyMap:
    resolution: float = 50.0
    width: int = 200
    height: int = 200
    origin_x: float = 0.0
    origin_y: float = 0.0
    grid: np.ndarray = field(default=None)
    log_odds: np.ndarray = field(default=None)

    OCCUPIED = 100
    FREE = 0
    UNKNOWN = -1

    LOG_ODDS_OCCUPIED = 0.9
    LOG_ODDS_FREE = 0.4

    def __post_init__(self):
        if self.grid is None:
            self.grid = np.full(
                (self.height, self.width), self.UNKNOWN, dtype=np.float32
            )
        if self.log_odds is None:
            self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin_x) / self.resolution + self.width / 2)
        gy = int((y - self.origin_y) / self.resolution + self.height / 2)
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = (gx - self.width / 2) * self.resolution + self.origin_x
        y = (gy - self.height / 2) * self.resolution + self.origin_y
        return (x, y)

    def is_valid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def ray_casting(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx / 2
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        cells.append((x1, y1))
        return cells

    def update_scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        scan_points: List[dict],
        max_range: float = 4000.0,
    ):
        rx, ry = self.world_to_grid(robot_x, robot_y)

        for point in scan_points:
            angle_deg = point.get("angle", 0)
            distance_mm = point.get("distance", 0)

            if distance_mm <= 0 or distance_mm > max_range:
                continue

            angle_rad = math.radians(angle_deg) + robot_yaw

            end_x = robot_x + distance_mm * math.cos(angle_rad)
            end_y = robot_y + distance_mm * math.sin(angle_rad)

            gx_end, gy_end = self.world_to_grid(end_x, end_y)

            ray_cells = self.ray_casting(rx, ry, gx_end, gy_end)

            for i, (cx, cy) in enumerate(ray_cells[:-1]):
                if self.is_valid(cx, cy):
                    self.log_odds[cy, cx] += self.LOG_ODDS_FREE

            if len(ray_cells) > 0 and distance_mm < max_range * 0.95:
                cx, cy = ray_cells[-1]
                if self.is_valid(cx, cy):
                    self.log_odds[cy, cx] += self.LOG_ODDS_OCCUPIED

        self._update_grid_from_log_odds()

    def _update_grid_from_log_odds(self):
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        self.grid = (prob * 100).astype(np.float32)
        self.grid[self.log_odds == 0] = self.UNKNOWN

    def get_probability(self, gx: int, gy: int) -> float:
        if not self.is_valid(gx, gy):
            return self.UNKNOWN
        return self.grid[gy, gx]

    def is_occupied(self, gx: int, gy: int, threshold: float = 50.0) -> bool:
        return self.get_probability(gx, gy) > threshold

    def is_free(self, gx: int, gy: int, threshold: float = 30.0) -> bool:
        prob = self.get_probability(gx, gy)
        return 0 <= prob < threshold

    def is_unknown(self, gx: int, gy: int) -> bool:
        return self.get_probability(gx, gy) == self.UNKNOWN

    def find_frontiers(self, min_frontier_size: int = 5) -> List[List[Tuple[int, int]]]:
        frontiers = []
        visited = np.zeros_like(self.grid, dtype=bool)

        for gy in range(1, self.height - 1):
            for gx in range(1, self.width - 1):
                if visited[gy, gx]:
                    continue

                if not self.is_free(gx, gy):
                    continue

                neighbors = [(gx - 1, gy), (gx + 1, gy), (gx, gy - 1), (gx, gy + 1)]

                has_unknown_neighbor = False
                for nx, ny in neighbors:
                    if self.is_unknown(nx, ny):
                        has_unknown_neighbor = True
                        break

                if has_unknown_neighbor:
                    frontier = self._grow_frontier(gx, gy, visited)
                    if len(frontier) >= min_frontier_size:
                        frontiers.append(frontier)

        return frontiers

    def _grow_frontier(
        self, start_x: int, start_y: int, visited: np.ndarray
    ) -> List[Tuple[int, int]]:
        frontier = []
        queue = [(start_x, start_y)]

        while queue:
            x, y = queue.pop(0)

            if visited[y, x]:
                continue

            if not self.is_free(x, y):
                continue

            neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]

            has_unknown = any(
                self.is_unknown(nx, ny) for nx, ny in neighbors if self.is_valid(nx, ny)
            )

            if has_unknown:
                visited[y, x] = True
                frontier.append((x, y))

                for nx, ny in neighbors:
                    if self.is_valid(nx, ny) and not visited[ny, nx]:
                        queue.append((nx, ny))

        return frontier

    def get_frontier_center(
        self, frontier: List[Tuple[int, int]]
    ) -> Tuple[float, float]:
        if not frontier:
            return (0.0, 0.0)

        sum_x = sum(p[0] for p in frontier)
        sum_y = sum(p[1] for p in frontier)

        center_gx = sum_x / len(frontier)
        center_gy = sum_y / len(frontier)

        return self.grid_to_world(int(center_gx), int(center_gy))

    def to_dict(self) -> dict:
        return {
            "resolution": self.resolution,
            "width": self.width,
            "height": self.height,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "grid": self.grid.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "OccupancyMap":
        return cls(
            resolution=data["resolution"],
            width=data["width"],
            height=data["height"],
            origin_x=data["origin_x"],
            origin_y=data["origin_y"],
            grid=np.array(data["grid"], dtype=np.float32),
            log_odds=np.zeros((data["height"], data["width"]), dtype=np.float32),
        )


def polar_to_cartesian(
    scan_points: List[dict],
    robot_x: float = 0,
    robot_y: float = 0,
    robot_yaw: float = 0,
) -> List[Tuple[float, float]]:
    cartesian_points = []

    for point in scan_points:
        angle_deg = point.get("angle", 0)
        distance_mm = point.get("distance", 0)

        if distance_mm <= 0:
            continue

        angle_rad = math.radians(angle_deg) + robot_yaw

        x = robot_x + distance_mm * math.cos(angle_rad)
        y = robot_y + distance_mm * math.sin(angle_rad)

        cartesian_points.append((x, y))

    return cartesian_points
