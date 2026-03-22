"""
Autonomous Room Exploration System
Based on: RELAX paper (https://arxiv.org/html/2309.08095v2)

Coordinates autonomous exploration using frontier-based algorithm
with RRT path planning and obstacle avoidance.
"""

import asyncio
import json
import math
import os
import sys
import time
from enum import Enum
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass, field

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
from database import get_connection

from occupancy_map import OccupancyMap, polar_to_cartesian
from path_planner import PathPlanner, Node


class ExplorerState(Enum):
    IDLE = "idle"
    SCANNING = "scanning"
    PLANNING = "planning"
    MOVING = "moving"
    COMPLETED = "completed"
    ERROR = "error"


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class ExplorationStats:
    total_scans: int = 0
    total_distance: float = 0.0
    area_explored: float = 0.0
    time_elapsed: float = 0.0
    frontiers_visited: int = 0


class AutoExplorer:
    def __init__(
        self, map_resolution: float = 50.0, map_size: Tuple[int, int] = (400, 400)
    ):
        self.occupancy_map = OccupancyMap(
            resolution=map_resolution, width=map_size[0], height=map_size[1]
        )
        self.path_planner = PathPlanner(self.occupancy_map)
        self.robot_pose = RobotPose()
        self.state = ExplorerState.IDLE
        self.stats = ExplorationStats()
        self.current_path: List[Tuple[float, float]] = []
        self.current_path_index = 0
        self.scan_callback: Optional[Callable] = None
        self.movement_callback: Optional[Callable] = None
        self.status_callback: Optional[Callable] = None
        self.running = False
        self.stop_requested = False

        self.move_speed = 100.0
        self.rotation_speed = 30.0
        self.scan_at_points = True
        self.min_frontier_size = 10

        self._last_pose = RobotPose()
        self._start_time = 0.0

    def set_callbacks(
        self,
        scan_callback: Callable = None,
        movement_callback: Callable = None,
        status_callback: Callable = None,
    ):
        self.scan_callback = scan_callback
        self.movement_callback = movement_callback
        self.status_callback = status_callback

    def update_robot_pose(self, x: float, y: float, yaw: float):
        if self._start_time == 0:
            self._start_time = time.time()

        dx = x - self._last_pose.x
        dy = y - self._last_pose.y
        self.stats.total_distance += math.sqrt(dx**2 + dy**2)

        self.robot_pose.x = x
        self.robot_pose.y = y
        self.robot_pose.yaw = yaw
        self._last_pose = RobotPose(x, y, yaw)

    def update_map(self, scan_points: List[dict]):
        self.occupancy_map.update_scan(
            self.robot_pose.x, self.robot_pose.y, self.robot_pose.yaw, scan_points
        )
        self.stats.total_scans += 1

        free_cells = sum(
            1
            for gy in range(self.occupancy_map.height)
            for gx in range(self.occupancy_map.width)
            if self.occupancy_map.grid[gy, gx] > 0
            and self.occupancy_map.grid[gy, gx] < 50
        )

        self.stats.area_explored = (
            free_cells * (self.occupancy_map.resolution**2) / 1_000_000
        )

    async def start_exploration(self):
        if self.state != ExplorerState.IDLE:
            return {"error": "Already exploring or in progress"}

        self.state = ExplorerState.SCANNING
        self.running = True
        self.stop_requested = False
        self._start_time = time.time()

        await self._run_exploration_loop()

        return {"status": "completed", "stats": self._get_stats_dict()}

    async def stop_exploration(self):
        self.stop_requested = True
        self.running = False
        self.state = ExplorerState.IDLE
        return {"status": "stopped", "stats": self._get_stats_dict()}

    async def _run_exploration_loop(self):
        while self.running and not self.stop_requested:
            try:
                if self.state == ExplorerState.SCANNING:
                    await self._do_scan()

                elif self.state == ExplorerState.PLANNING:
                    await self._do_planning()

                elif self.state == ExplorerState.MOVING:
                    await self._do_movement()

                elif self.state == ExplorerState.COMPLETED:
                    break

                elif self.state == ExplorerState.ERROR:
                    break

            except Exception as e:
                print(f"[AutoExplorer] Error: {e}")
                self.state = ExplorerState.ERROR
                await self._notify_status({"error": str(e)})
                break

        self.stats.time_elapsed = time.time() - self._start_time
        await self._notify_status(self._get_status())

    async def _do_scan(self):
        if self.scan_callback:
            scan_data = await self.scan_callback()
            if scan_data:
                self.update_map(scan_data)

        self.state = ExplorerState.PLANNING
        await self._notify_status(self._get_status())

    async def _do_planning(self):
        frontiers = self.occupancy_map.find_frontiers(self.min_frontier_size)

        if not frontiers:
            self.state = ExplorerState.COMPLETED
            return

        goal = self.path_planner.find_frontier_goal(
            frontiers, (self.robot_pose.x, self.robot_pose.y)
        )

        if goal is None:
            self.state = ExplorerState.COMPLETED
            return

        path = self.path_planner.plan_rrt((self.robot_pose.x, self.robot_pose.y), goal)

        if path:
            self.current_path = path
            self.current_path_index = 1
            self.stats.frontiers_visited += 1
            self.state = ExplorerState.MOVING
        else:
            self.state = ExplorerState.SCANNING

        await self._notify_status(self._get_status())

    async def _do_movement(self):
        if not self.current_path or self.current_path_index >= len(self.current_path):
            self.state = ExplorerState.SCANNING
            return

        target_x, target_y = self.current_path[self.current_path_index]

        target_yaw = math.atan2(
            target_y - self.robot_pose.y, target_x - self.robot_pose.x
        )

        yaw_diff = self._normalize_angle(target_yaw - self.robot_pose.yaw)

        if abs(yaw_diff) > math.radians(5):
            rotation = max(
                -self.rotation_speed,
                min(self.rotation_speed, math.degrees(yaw_diff) * 0.5),
            )
            if self.movement_callback:
                await self.movement_callback(linear_x=0, angular_z=rotation)
            await asyncio.sleep(0.1)
            return

        distance = math.sqrt(
            (target_x - self.robot_pose.x) ** 2 + (target_y - self.robot_pose.y) ** 2
        )

        if distance < self.move_speed:
            self.current_path_index += 1

            if self.current_path_index >= len(self.current_path):
                self.state = ExplorerState.SCANNING
            return

        speed = min(self.move_speed, distance * 0.5)

        if self.movement_callback:
            await self.movement_callback(linear_x=speed, angular_z=0)

        await asyncio.sleep(0.1)
        await self._notify_status(self._get_status())

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _get_status(self) -> dict:
        return {
            "state": self.state.value,
            "robot_pose": {
                "x": self.robot_pose.x,
                "y": self.robot_pose.y,
                "yaw": self.robot_pose.yaw,
            },
            "current_path": self.current_path[self.current_path_index :]
            if self.current_path
            else [],
            "stats": self._get_stats_dict(),
        }

    def _get_stats_dict(self) -> dict:
        return {
            "total_scans": self.stats.total_scans,
            "total_distance": round(self.stats.total_distance, 2),
            "area_explored_m2": round(self.stats.area_explored, 2),
            "time_elapsed": round(self.stats.time_elapsed, 2),
            "frontiers_visited": self.stats.frontiers_visited,
        }

    async def _notify_status(self, status: dict):
        if self.status_callback:
            await self.status_callback(status)

    def get_map_data(self) -> dict:
        return self.occupancy_map.to_dict()

    def get_exploration_status(self) -> dict:
        return self._get_status()

    def save_map_to_db(self, map_name: str = None):
        if map_name is None:
            map_name = f"exploration_{int(time.time())}"

        conn = get_connection()
        c = conn.cursor()
        c.execute(
            "INSERT INTO slam_map (map_name, points_json) VALUES (?, ?)",
            (map_name, json.dumps(self.occupancy_map.to_dict())),
        )
        conn.commit()
        conn.close()

        return {"saved": True, "map_name": map_name}


class SimulatedRobot:
    def __init__(self, room_config: dict = None):
        self.pose = RobotPose()
        self.room_config = room_config or self._default_room()

    def _default_room(self) -> dict:
        return {
            "walls": [
                {"angle_start": 0, "angle_end": 50, "distance": 3000},
                {"angle_start": 50, "angle_end": 130, "distance": 4500},
                {"angle_start": 130, "angle_end": 200, "distance": 3500},
                {"angle_start": 200, "angle_end": 280, "distance": 4000},
                {"angle_start": 280, "angle_end": 340, "distance": 3200},
                {"angle_start": 340, "angle_end": 360, "distance": 3000},
            ],
            "obstacles": [
                {"angle": 30, "width": 20, "distance": 1500},
                {"angle": 100, "width": 30, "distance": 2000},
                {"angle": 180, "width": 25, "distance": 1800},
                {"angle": 250, "width": 15, "distance": 1200},
                {"angle": 310, "width": 20, "distance": 1600},
            ],
        }

    def simulate_scan(self, noise: float = 20.0) -> List[dict]:
        scan_points = []

        for angle in range(0, 360, 2):
            wall_dist = 5000

            for wall in self.room_config["walls"]:
                angle_start = wall["angle_start"]
                angle_end = wall["angle_end"]

                if angle_start <= angle < angle_end:
                    wall_dist = wall["distance"]
                    break

                if angle_start > angle_end:
                    if angle >= angle_start or angle < angle_end:
                        wall_dist = wall["distance"]
                        break

            for obs in self.room_config["obstacles"]:
                angle_diff = abs(angle - obs["angle"])
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff

                if angle_diff < obs["width"]:
                    wall_dist = min(wall_dist, obs["distance"])

            noise_val = (hash(f"{angle}{time.time()}") % int(noise * 2)) - noise
            distance = max(100, wall_dist + noise_val)

            angle_rad = math.radians(angle) + self.pose.yaw
            world_x = self.pose.x + distance * math.cos(angle_rad)
            world_y = self.pose.y + distance * math.sin(angle_rad)

            scan_points.append(
                {
                    "angle": angle,
                    "distance": int(distance),
                    "world_x": world_x,
                    "world_y": world_y,
                }
            )

        return scan_points

    def move(self, linear_x: float, angular_z: float, dt: float = 0.1):
        self.pose.yaw += math.radians(angular_z * dt)
        self.pose.x += linear_x * dt * math.cos(self.pose.yaw)
        self.pose.y += linear_x * dt * math.sin(self.pose.yaw)

        return {"x": self.pose.x, "y": self.pose.y, "yaw": self.pose.yaw}

    def set_pose(self, x: float, y: float, yaw: float = None):
        self.pose.x = x
        self.pose.y = y
        if yaw is not None:
            self.pose.yaw = yaw
