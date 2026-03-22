"""
Path Planner with RRT (Rapidly Exploring Random Tree)
Based on: RELAX paper (https://arxiv.org/html/2309.08095v2)

Implements path planning algorithms for autonomous navigation.
"""

import numpy as np
import math
import random
from typing import List, Tuple, Optional
from dataclasses import dataclass
from occupancy_map import OccupancyMap


@dataclass
class Node:
    x: float
    y: float
    parent: Optional["Node"] = None
    cost: float = 0.0

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return abs(self.x - other.x) < 1e-6 and abs(self.y - other.y) < 1e-6


class PathPlanner:
    def __init__(self, occupancy_map: OccupancyMap):
        self.map = occupancy_map
        self.goal_bias = 0.1
        self.max_iterations = 10000
        self.step_size = 100.0
        self.goal_threshold = 50.0
        self.robot_radius = 150.0

    def euclidean_distance(self, n1: Node, n2: Node) -> float:
        return math.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)

    def is_collision_free(self, x: float, y: float) -> bool:
        gx, gy = self.map.world_to_grid(x, y)

        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if self.map.is_occupied(gx + dx, gy + dy):
                    return False

        return True

    def is_path_collision_free(self, n1: Node, n2: Node) -> bool:
        dist = self.euclidean_distance(n1, n2)
        steps = int(dist / (self.robot_radius / 2)) + 1

        for i in range(steps + 1):
            t = i / steps
            x = n1.x + t * (n2.x - n1.x)
            y = n1.y + t * (n2.y - n1.y)

            if not self.is_collision_free(x, y):
                return False

        return True

    def get_random_node(self, goal: Node) -> Node:
        if random.random() < self.goal_bias:
            return Node(goal.x, goal.y)

        x = random.uniform(
            self.map.origin_x - self.map.width * self.map.resolution / 2,
            self.map.origin_x + self.map.width * self.map.resolution / 2,
        )
        y = random.uniform(
            self.map.origin_y - self.map.height * self.map.resolution / 2,
            self.map.origin_y + self.map.height * self.map.resolution / 2,
        )

        return Node(x, y)

    def get_nearest_node(self, nodes: List[Node], random_node: Node) -> Node:
        min_dist = float("inf")
        nearest = None

        for node in nodes:
            dist = self.euclidean_distance(node, random_node)
            if dist < min_dist:
                min_dist = dist
                nearest = node

        return nearest

    def steer(self, from_node: Node, to_node: Node) -> Node:
        dist = self.euclidean_distance(from_node, to_node)

        if dist <= self.step_size:
            return Node(to_node.x, to_node.y, from_node)

        ratio = self.step_size / dist
        new_x = from_node.x + ratio * (to_node.x - from_node.x)
        new_y = from_node.y + ratio * (to_node.y - from_node.y)

        return Node(new_x, new_y, from_node)

    def get_neighbors(self, nodes: List[Node], node: Node, radius: float) -> List[Node]:
        neighbors = []
        for n in nodes:
            if self.euclidean_distance(n, node) < radius:
                neighbors.append(n)
        return neighbors

    def extract_path(self, goal_node: Node) -> List[Tuple[float, float]]:
        path = []
        node = goal_node

        while node is not None:
            path.append((node.x, node.y))
            node = node.parent

        return list(reversed(path))

    def smooth_path(
        self, path: List[Tuple[float, float]], iterations: int = 50
    ) -> List[Tuple[float, float]]:
        if len(path) <= 2:
            return path

        smoothed = path.copy()

        for _ in range(iterations):
            if len(smoothed) <= 2:
                break

            i = random.randint(0, len(smoothed) - 2)
            j = random.randint(i + 1, len(smoothed) - 1)

            if i == j:
                continue

            node_i = Node(smoothed[i][0], smoothed[i][1])
            node_j = Node(smoothed[j][0], smoothed[j][1])

            if self.is_path_collision_free(node_i, node_j):
                smoothed = smoothed[: i + 1] + smoothed[j:]

        return smoothed

    def plan_rrt(
        self, start: Tuple[float, float], goal: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])

        if not self.is_collision_free(start[0], start[1]):
            print("[PathPlanner] Start position is in collision!")
            return None

        if not self.is_collision_free(goal[0], goal[1]):
            print("[PathPlanner] Goal position is in collision!")
            return None

        nodes = [start_node]

        for iteration in range(self.max_iterations):
            random_node = self.get_random_node(goal_node)
            nearest_node = self.get_nearest_node(nodes, random_node)
            new_node = self.steer(nearest_node, random_node)

            if not self.is_path_collision_free(nearest_node, new_node):
                continue

            new_node.cost = nearest_node.cost + self.euclidean_distance(
                nearest_node, new_node
            )
            nodes.append(new_node)

            if self.euclidean_distance(new_node, goal_node) < self.goal_threshold:
                final_node = Node(goal_node.x, goal_node.y, new_node)
                final_node.cost = new_node.cost + self.euclidean_distance(
                    new_node, goal_node
                )
                path = self.extract_path(final_node)
                return self.smooth_path(path)

            if iteration % 1000 == 0:
                print(f"[PathPlanner] RRT iteration {iteration}, nodes: {len(nodes)}")

        print("[PathPlanner] No path found!")
        return None

    def plan_rrt_star(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        rewiring_radius: float = 200.0,
    ) -> Optional[List[Tuple[float, float]]]:
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])

        if not self.is_collision_free(start[0], start[1]):
            return None

        if not self.is_collision_free(goal[0], goal[1]):
            return None

        nodes = [start_node]

        for iteration in range(self.max_iterations):
            random_node = self.get_random_node(goal_node)
            nearest_node = self.get_nearest_node(nodes, random_node)
            new_node = self.steer(nearest_node, random_node)

            if not self.is_path_collision_free(nearest_node, new_node):
                continue

            new_node.cost = nearest_node.cost + self.euclidean_distance(
                nearest_node, new_node
            )

            neighbors = self.get_neighbors(nodes, new_node, rewiring_radius)

            for neighbor in neighbors:
                if neighbor == nearest_node:
                    continue

                potential_cost = neighbor.cost + self.euclidean_distance(
                    neighbor, new_node
                )
                if potential_cost < new_node.cost:
                    if self.is_path_collision_free(neighbor, new_node):
                        new_node.parent = neighbor
                        new_node.cost = potential_cost

            for neighbor in neighbors:
                potential_cost = new_node.cost + self.euclidean_distance(
                    new_node, neighbor
                )
                if potential_cost < neighbor.cost:
                    if self.is_path_collision_free(new_node, neighbor):
                        neighbor.parent = new_node
                        neighbor.cost = potential_cost

            nodes.append(new_node)

            if self.euclidean_distance(new_node, goal_node) < self.goal_threshold:
                final_node = Node(goal_node.x, goal_node.y, new_node)
                final_node.cost = new_node.cost + self.euclidean_distance(
                    new_node, goal_node
                )
                path = self.extract_path(final_node)
                return self.smooth_path(path)

            if iteration % 1000 == 0:
                print(f"[PathPlanner] RRT* iteration {iteration}, nodes: {len(nodes)}")

        return None

    def find_frontier_goal(
        self, frontiers: List[List[Tuple[int, int]]], robot_pos: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        if not frontiers:
            return None

        best_frontier = None
        best_score = float("inf")

        for frontier in frontiers:
            center = self.map.get_frontier_center(frontier)

            score = math.sqrt(
                (center[0] - robot_pos[0]) ** 2 + (center[1] - robot_pos[1]) ** 2
            )
            score += len(frontier) * -0.5

            if score < best_score:
                best_score = score
                best_frontier = frontier

        if best_frontier:
            return self.map.get_frontier_center(best_frontier)

        return None
