"""
Go-to-Goal Navigation with 2D LiDAR
Navigates robot from current position to specified goal coordinates.
Based on: RRT path planning with obstacle avoidance using LiDAR

Usage:
    cd research/autonomous_mapping
    python goto_goal.py --goal-x 1000 --goal-y 2000

    # Or interactive mode:
    python goto_goal.py --interactive
"""

import asyncio
import json
import math
import os
import sys
import time
import struct
import threading
import argparse
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from occupancy_map import OccupancyMap
from path_planner import PathPlanner

try:
    import serial

    LIDAR_AVAILABLE = True
except ImportError:
    LIDAR_AVAILABLE = False
    print("[!] pyserial not installed. Run: pip install pyserial")

# ===== CONFIGURATION =====
LIDAR_PORT = "/dev/cu.usbserial-0001"
LIDAR_BAUD = 115200
ROBOT_API = "http://localhost:8000"

# Movement parameters
FORWARD_POWER = 2000
TURN_POWER = 1500
MOVE_DURATION = 0.3
TURN_DURATION = 0.2

# Navigation parameters
GOAL_TOLERANCE = 150.0  # mm
WAYPOINT_TOLERANCE = 100.0  # mm
OBSTACLE_STOP_DISTANCE = 300.0  # mm - stop if obstacle closer than this
PATH_REPLAN_INTERVAL = 5  # re-plan every N waypoints
MAX_RETRIES = 3  # max retries for path planning


class NavState(Enum):
    IDLE = "idle"
    SCANNING = "scanning"
    PLANNING = "planning"
    MOVING = "moving"
    REACHED = "reached"
    OBSTACLE_DETECTED = "obstacle_detected"
    REPLANNING = "replanning"
    ERROR = "error"


@dataclass
class NavigationConfig:
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: Optional[float] = None  # Optional final orientation
    use_lidar: bool = True
    simulation_mode: bool = True
    verbose: bool = True
    save_map: bool = True


class LidarReader:
    """YDLIDAR X2 Reader - same as in server.py"""

    HEADER = bytes([0xAA, 0x55])

    def __init__(self, port=LIDAR_PORT, baudrate=LIDAR_BAUD):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False

    def connect(self) -> bool:
        if not LIDAR_AVAILABLE:
            return False
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
            )
            self.connected = True
            print(f"[Lidar] Connected: {self.port}")
            return True
        except Exception as e:
            print(f"[Lidar] Connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self):
        if self.serial and self.connected:
            self.serial.close()
            self.connected = False

    @staticmethod
    def calc_angle(raw_angle):
        return (raw_angle >> 1) / 64.0

    def read_packet(self) -> Optional[bytes]:
        if not self.connected or not self.serial:
            return None
        while True:
            b = self.serial.read(1)
            if not b:
                return None
            if b[0] == 0xAA:
                b2 = self.serial.read(1)
                if b2 and b2[0] == 0x55:
                    break
        header = self.serial.read(8)
        if len(header) < 8:
            return None
        lsn = header[1]
        data_len = lsn * 2
        sample_data = self.serial.read(data_len)
        if len(sample_data) < data_len:
            return None
        return header + sample_data

    def parse_packet(self, data: bytes) -> Optional[dict]:
        if len(data) < 10:
            return None
        ct = data[0]
        lsn = data[1]
        fsa = struct.unpack_from("<H", data, 2)[0]
        lsa = struct.unpack_from("<H", data, 4)[0]
        start_angle = self.calc_angle(fsa)
        end_angle = self.calc_angle(lsa)
        if end_angle < start_angle:
            angle_diff = (360.0 - start_angle) + end_angle
        else:
            angle_diff = end_angle - start_angle
        sample_data = data[8:]
        points = []
        for i in range(lsn):
            distance = struct.unpack_from("<H", sample_data, i * 2)[0]
            if lsn > 1:
                angle = start_angle + (angle_diff / (lsn - 1)) * i
            else:
                angle = start_angle
            angle = angle % 360.0
            points.append({"angle": round(angle, 2), "distance": distance})
        is_start = (ct & 0x01) == 1
        return {"points": points, "is_start": is_start}

    def scan_360(self) -> Optional[List[dict]]:
        """Get a complete 360° scan"""
        if not self.connected:
            return None
        scan_points = []
        scan_complete = False
        start_time = time.time()
        while not scan_complete and (time.time() - start_time) < 5.0:  # 5s timeout
            raw = self.read_packet()
            if raw is None:
                continue
            result = self.parse_packet(raw)
            if result is None:
                continue
            if result["is_start"] and scan_points:
                scan_complete = True
            else:
                scan_points.extend(result["points"])
        return scan_points if scan_complete else None


class RobotController:
    """Robot control interface - can use real robot or simulation"""

    def __init__(self, simulation_mode: bool = True):
        self.simulation_mode = simulation_mode
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def check_connection(self) -> bool:
        """Check if robot is connected"""
        if self.simulation_mode:
            return True
        try:
            import urllib.request

            req = urllib.request.Request(f"{ROBOT_API}/api/status", method="GET")
            with urllib.request.urlopen(req, timeout=2) as response:
                data = json.loads(response.read().decode())
                return data.get("connected") == "true"
        except:
            return False

    async def send_command(
        self, action: str, power: int = None, duration: float = None
    ) -> bool:
        """Send movement command to robot"""
        if self.simulation_mode:
            # Simulate movement
            if action == "forward":
                dist = (power or FORWARD_POWER) * (duration or MOVE_DURATION) * 0.1
                self.pose["x"] += dist * math.cos(self.pose["yaw"])
                self.pose["y"] += dist * math.sin(self.pose["yaw"])
            elif action == "backward":
                dist = (power or FORWARD_POWER) * (duration or MOVE_DURATION) * 0.1
                self.pose["x"] -= dist * math.cos(self.pose["yaw"])
                self.pose["y"] -= dist * math.sin(self.pose["yaw"])
            elif action == "left":
                angle = (power or TURN_POWER) * (duration or TURN_DURATION) * 0.5
                self.pose["yaw"] += math.radians(angle)
            elif action == "right":
                angle = (power or TURN_POWER) * (duration or TURN_DURATION) * 0.5
                self.pose["yaw"] -= math.radians(angle)

            # Normalize yaw
            while self.pose["yaw"] > math.pi:
                self.pose["yaw"] -= 2 * math.pi
            while self.pose["yaw"] < -math.pi:
                self.pose["yaw"] += 2 * math.pi

            return True
        else:
            # Real robot API call
            try:
                import urllib.request

                data = {"action": action}
                if power:
                    data["power"] = power
                if duration:
                    data["duration"] = duration

                req = urllib.request.Request(
                    f"{ROBOT_API}/api/move",
                    data=json.dumps(data).encode(),
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=5):
                    return True
            except Exception as e:
                print(f"[Robot] Command failed: {e}")
                return False

    def get_pose(self) -> dict:
        """Get current robot pose"""
        return self.pose.copy()

    def set_pose(self, x: float, y: float, yaw: float = None):
        """Set robot pose (for calibration)"""
        self.pose["x"] = x
        self.pose["y"] = y
        if yaw is not None:
            self.pose["yaw"] = yaw


class GotoGoalNavigator:
    """Main navigation controller for go-to-goal behavior"""

    def __init__(self, config: NavigationConfig):
        self.config = config
        self.state = NavState.IDLE

        # Initialize components
        self.occupancy_map = OccupancyMap(resolution=50.0, width=400, height=400)
        self.path_planner = PathPlanner(self.occupancy_map)
        self.lidar = LidarReader()
        self.robot = RobotController(simulation_mode=config.simulation_mode)

        # Navigation state
        self.current_path: List[Tuple[float, float]] = []
        self.path_index = 0
        self.goal_reached = False
        self.obstacles_in_front = []

        # Statistics
        self.stats = {
            "start_time": None,
            "total_distance": 0.0,
            "waypoints_reached": 0,
            "path_replans": 0,
            "obstacles_detected": 0,
        }

        self._last_pose = None

    def connect_sensors(self) -> bool:
        """Connect to LiDAR and robot"""
        print("[Init] Connecting sensors...")

        # Try to connect to LiDAR if requested
        if self.config.use_lidar:
            if self.lidar.connect():
                print("[Init] LiDAR connected")
            else:
                print("[Init] LiDAR not available, will use simulation")

        # Check robot connection
        if not self.config.simulation_mode:
            if self.robot.check_connection():
                print("[Init] Robot connected")
            else:
                print("[Init] Robot not connected, switching to simulation")
                self.config.simulation_mode = True
                self.robot.simulation_mode = True
        else:
            print("[Init] Running in simulation mode")

        return True

    async def scan_environment(self) -> List[dict]:
        """Perform 360° LiDAR scan and update map"""
        self.state = NavState.SCANNING

        if self.lidar.connected:
            scan_points = self.lidar.scan_360()
            if scan_points:
                pose = self.robot.get_pose()
                # Add world coordinates to scan points
                for p in scan_points:
                    angle_rad = math.radians(p["angle"])
                    p["world_x"] = pose["x"] + p["distance"] * math.cos(
                        angle_rad + pose["yaw"]
                    )
                    p["world_y"] = pose["y"] + p["distance"] * math.sin(
                        angle_rad + pose["yaw"]
                    )

                # Update occupancy map
                self.occupancy_map.update_scan(
                    pose["x"], pose["y"], pose["yaw"], scan_points
                )
                return scan_points

        # Return empty scan if LiDAR not available
        return []

    def detect_obstacles_front(
        self, scan_points: List[dict], angle_range: float = 45.0
    ) -> List[dict]:
        """Detect obstacles in front of robot within angle range"""
        if not scan_points:
            return []

        obstacles = []
        for p in scan_points:
            angle = p["angle"]
            distance = p.get("distance", 0)

            # Check if in front sector (-angle_range/2 to +angle_range/2)
            # Angle 0 is front, 90 is left, -90 is right (in robot frame)
            angle_diff = ((angle + 180) % 360) - 180  # Normalize to -180 to 180

            if (
                abs(angle_diff) < angle_range / 2
                and 0 < distance < OBSTACLE_STOP_DISTANCE
            ):
                obstacles.append(p)

        return obstacles

    def plan_path(
        self, goal_x: float, goal_y: float
    ) -> Optional[List[Tuple[float, float]]]:
        """Plan path from current position to goal"""
        self.state = NavState.PLANNING

        pose = self.robot.get_pose()
        start = (pose["x"], pose["y"])
        goal = (goal_x, goal_y)

        print(
            f"[Planner] Planning from ({start[0]:.0f}, {start[1]:.0f}) to ({goal[0]:.0f}, {goal[1]:.0f})"
        )

        path = self.path_planner.plan_rrt(start, goal)

        if path and len(path) >= 2:
            # Simplify path by removing intermediate points that are too close
            simplified = [path[0]]
            for i in range(1, len(path)):
                dx = path[i][0] - simplified[-1][0]
                dy = path[i][1] - simplified[-1][1]
                if math.sqrt(dx * dx + dy * dy) > 200 or i == len(path) - 1:
                    simplified.append(path[i])

            print(f"[Planner] Path found: {len(simplified)} waypoints")
            return simplified
        else:
            print("[Planner] No path found!")
            return None

    async def rotate_to_yaw(self, target_yaw: float, tolerance: float = 0.1) -> bool:
        """Rotate robot to face target yaw"""
        pose = self.robot.get_pose()
        yaw_diff = target_yaw - pose["yaw"]

        # Normalize to -pi to pi
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi

        if abs(yaw_diff) < tolerance:
            return True

        # Determine turn direction
        action = "left" if yaw_diff > 0 else "right"
        turn_duration = abs(yaw_diff) / (
            math.radians(TURN_POWER * 0.5)
        )  # rough estimate
        turn_duration = min(turn_duration, 1.0)  # max 1 second per adjustment

        print(f"[Rotate] Turning {math.degrees(yaw_diff):.1f}° {action}")
        await self.robot.send_command(action, TURN_POWER, turn_duration)
        await asyncio.sleep(0.2)

        return False

    async def move_to_waypoint(self, target_x: float, target_y: float) -> bool:
        """Move robot to a specific waypoint"""
        pose = self.robot.get_pose()

        dx = target_x - pose["x"]
        dy = target_y - pose["y"]
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < WAYPOINT_TOLERANCE:
            return True  # Reached waypoint

        # Calculate target yaw
        target_yaw = math.atan2(dy, dx)

        # First, rotate to face target
        yaw_diff = target_yaw - pose["yaw"]
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi

        if abs(yaw_diff) > 0.2:  # ~11 degrees
            await self.rotate_to_yaw(target_yaw)
            return False

        # Check for obstacles before moving
        scan_points = await self.scan_environment()
        obstacles = self.detect_obstacles_front(scan_points)

        if obstacles:
            closest = min(obstacles, key=lambda p: p["distance"])
            print(
                f"[Obstacle] Detected at {closest['distance']}mm, angle {closest['angle']:.1f}°"
            )
            self.state = NavState.OBSTACLE_DETECTED
            self.stats["obstacles_detected"] += 1
            return False

        # Move forward
        move_duration = min(distance / (FORWARD_POWER * 0.1), 1.0)  # cap at 1s
        print(f"[Move] Forward {distance:.0f}mm")
        await self.robot.send_command("forward", FORWARD_POWER, move_duration)

        # Update statistics
        if self._last_pose:
            dx = pose["x"] - self._last_pose["x"]
            dy = pose["y"] - self._last_pose["y"]
            self.stats["total_distance"] += math.sqrt(dx * dx + dy * dy)
        self._last_pose = pose.copy()

        await asyncio.sleep(0.1)
        return False

    async def navigate(
        self, goal_x: float, goal_y: float, goal_yaw: float = None
    ) -> dict:
        """Main navigation loop to reach goal"""
        self.config.goal_x = goal_x
        self.config.goal_y = goal_y
        self.config.goal_yaw = goal_yaw

        print(f"\n{'=' * 60}")
        print(f"  GO-TO-GOAL NAVIGATION")
        print(f"  Goal: ({goal_x:.0f}, {goal_y:.0f}) mm")
        if goal_yaw:
            print(f"  Final yaw: {math.degrees(goal_yaw):.1f}°")
        print(f"{'=' * 60}\n")

        self.stats["start_time"] = time.time()

        # Initial scan
        print("[Nav] Initial environment scan...")
        await self.scan_environment()

        # Plan initial path
        print("[Nav] Planning path...")
        self.current_path = self.plan_path(goal_x, goal_y)

        if not self.current_path:
            return {
                "success": False,
                "error": "Path planning failed",
                "stats": self.stats,
            }

        self.path_index = 1  # Start from second point (first is current pos)
        self.state = NavState.MOVING

        # Main navigation loop
        while self.state != NavState.REACHED and self.state != NavState.ERROR:
            pose = self.robot.get_pose()

            # Check if goal reached
            dx = goal_x - pose["x"]
            dy = goal_y - pose["y"]
            distance_to_goal = math.sqrt(dx * dx + dy * dy)

            if distance_to_goal < GOAL_TOLERANCE:
                print(f"\n[Nav] Goal reached! Distance: {distance_to_goal:.0f}mm")
                self.state = NavState.REACHED
                break

            # Check if current waypoint reached
            if self.path_index < len(self.current_path):
                target_x, target_y = self.current_path[self.path_index]
                wp_dx = target_x - pose["x"]
                wp_dy = target_y - pose["y"]
                wp_dist = math.sqrt(wp_dx * wp_dx + wp_dy * wp_dy)

                if wp_dist < WAYPOINT_TOLERANCE:
                    print(
                        f"[Nav] Waypoint {self.path_index}/{len(self.current_path)} reached"
                    )
                    self.path_index += 1
                    self.stats["waypoints_reached"] += 1

                    # Periodically re-plan path
                    if self.path_index % PATH_REPLAN_INTERVAL == 0:
                        print("[Nav] Re-planning path...")
                        self.state = NavState.REPLANNING
                        new_path = self.plan_path(goal_x, goal_y)
                        if new_path:
                            self.current_path = new_path
                            self.path_index = 1
                            self.stats["path_replans"] += 1
                        self.state = NavState.MOVING

                    continue

                # Move to current waypoint
                reached = await self.move_to_waypoint(target_x, target_y)

                # Handle obstacle detection
                if self.state == NavState.OBSTACLE_DETECTED:
                    print("[Nav] Handling obstacle...")
                    await self.handle_obstacle()
            else:
                # No more waypoints but goal not reached - replan
                print("[Nav] Path ended but goal not reached, re-planning...")
                new_path = self.plan_path(goal_x, goal_y)
                if new_path:
                    self.current_path = new_path
                    self.path_index = 1
                    self.stats["path_replans"] += 1
                else:
                    print("[Nav] Failed to find new path!")
                    self.state = NavState.ERROR
                    break

        # Final orientation adjustment if requested
        if self.state == NavState.REACHED and goal_yaw is not None:
            print("[Nav] Adjusting final orientation...")
            for _ in range(10):  # Max 10 attempts
                if await self.rotate_to_yaw(goal_yaw, tolerance=0.05):
                    break
                await asyncio.sleep(0.1)

        # Calculate final stats
        elapsed = time.time() - self.stats["start_time"]
        self.stats["time_elapsed"] = elapsed

        print(f"\n{'=' * 60}")
        print(f"  NAVIGATION COMPLETE")
        print(f"  Status: {self.state.value}")
        print(f"  Time: {elapsed:.1f}s")
        print(f"  Distance: {self.stats['total_distance']:.0f}mm")
        print(f"  Waypoints: {self.stats['waypoints_reached']}")
        print(f"  Re-plans: {self.stats['path_replans']}")
        print(f"  Obstacles: {self.stats['obstacles_detected']}")
        print(f"{'=' * 60}\n")

        return {
            "success": self.state == NavState.REACHED,
            "state": self.state.value,
            "stats": self.stats,
            "final_pose": self.robot.get_pose(),
        }

    async def handle_obstacle(self):
        """Handle obstacle detection - simple avoidance"""
        print("[Avoid] Stopping and scanning...")
        await self.robot.send_command("stop")
        await asyncio.sleep(0.3)

        # Scan to find clear direction
        scan_points = await self.scan_environment()

        # Find clearest sector (left, front, right)
        sectors = {"left": [], "front": [], "right": []}

        for p in scan_points:
            angle = ((p["angle"] + 180) % 360) - 180
            dist = p.get("distance", 0)

            if -30 < angle < 30:
                sectors["front"].append(dist)
            elif angle >= 30:
                sectors["left"].append(dist)
            else:
                sectors["right"].append(dist)

        # Find best direction
        best_dir = None
        best_dist = 0
        for direction, distances in sectors.items():
            if distances:
                avg_dist = sum(distances) / len(distances)
                if avg_dist > best_dist:
                    best_dist = avg_dist
                    best_dir = direction

        if best_dir and best_dist > OBSTACLE_STOP_DISTANCE:
            print(f"[Avoid] Turning {best_dir} (clearance: {best_dist:.0f}mm)")
            if best_dir == "left":
                await self.robot.send_command("left", TURN_POWER, 0.5)
            elif best_dir == "right":
                await self.robot.send_command("right", TURN_POWER, 0.5)
            await asyncio.sleep(0.3)
        else:
            print("[Avoid] No clear path, backing up...")
            await self.robot.send_command("backward", FORWARD_POWER, 0.5)
            await asyncio.sleep(0.3)

        # Re-plan path
        print("[Avoid] Re-planning path...")
        new_path = self.plan_path(self.config.goal_x, self.config.goal_y)
        if new_path:
            self.current_path = new_path
            self.path_index = 1
            self.stats["path_replans"] += 1

        self.state = NavState.MOVING

    def save_map(self, filename: str = None):
        """Save current map to file"""
        if not filename:
            filename = f"goto_goal_map_{int(time.time())}.json"

        filepath = os.path.join(os.path.dirname(__file__), "maps", filename)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        with open(filepath, "w") as f:
            json.dump(
                {
                    "map": self.occupancy_map.to_dict(),
                    "path": self.current_path,
                    "goal": [self.config.goal_x, self.config.goal_y],
                    "stats": self.stats,
                },
                f,
                indent=2,
            )

        print(f"[Map] Saved to {filepath}")
        return filepath

    def cleanup(self):
        """Cleanup resources"""
        self.lidar.disconnect()
        print("[Cleanup] LiDAR disconnected")


async def interactive_mode():
    """Interactive mode for testing navigation"""
    print("\n" + "=" * 60)
    print("  GO-TO-GOAL NAVIGATION - INTERACTIVE MODE")
    print("=" * 60)
    print("\nCommands:")
    print("  goto x y [yaw]  - Navigate to goal (mm, degrees)")
    print("  scan            - Perform 360° scan")
    print("  pose            - Show current pose")
    print("  set x y [yaw]   - Set robot position")
    print("  map             - Save current map")
    print("  quit            - Exit")
    print("=" * 60 + "\n")

    config = NavigationConfig(simulation_mode=True, verbose=True)
    navigator = GotoGoalNavigator(config)
    navigator.connect_sensors()

    try:
        while True:
            cmd = input("> ").strip().lower()

            if not cmd:
                continue

            parts = cmd.split()
            action = parts[0]

            if action == "quit" or action == "exit":
                break

            elif action == "goto":
                if len(parts) < 3:
                    print("Usage: goto x y [yaw]")
                    continue
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    yaw = math.radians(float(parts[3])) if len(parts) > 3 else None
                    result = await navigator.navigate(x, y, yaw)
                    print(f"Result: {'SUCCESS' if result['success'] else 'FAILED'}")
                except ValueError:
                    print("Invalid coordinates")

            elif action == "scan":
                points = await navigator.scan_environment()
                print(f"Scan complete: {len(points)} points")

            elif action == "pose":
                pose = navigator.robot.get_pose()
                print(
                    f"Pose: x={pose['x']:.0f}, y={pose['y']:.0f}, yaw={math.degrees(pose['yaw']):.1f}°"
                )

            elif action == "set":
                if len(parts) < 3:
                    print("Usage: set x y [yaw]")
                    continue
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    yaw = math.radians(float(parts[3])) if len(parts) > 3 else 0
                    navigator.robot.set_pose(x, y, yaw)
                    print(f"Pose set to: ({x:.0f}, {y:.0f}, {math.degrees(yaw):.1f}°)")
                except ValueError:
                    print("Invalid coordinates")

            elif action == "map":
                navigator.save_map()

            else:
                print(f"Unknown command: {action}")

    except KeyboardInterrupt:
        print("\n[!] Interrupted")
    finally:
        navigator.cleanup()


async def main():
    parser = argparse.ArgumentParser(description="Go-to-Goal Navigation with LiDAR")
    parser.add_argument("--goal-x", type=float, help="Goal X coordinate (mm)")
    parser.add_argument("--goal-y", type=float, help="Goal Y coordinate (mm)")
    parser.add_argument("--goal-yaw", type=float, help="Goal orientation (degrees)")
    parser.add_argument("--interactive", action="store_true", help="Interactive mode")
    parser.add_argument(
        "--simulation", action="store_true", help="Force simulation mode"
    )
    parser.add_argument("--no-lidar", action="store_true", help="Disable LiDAR")
    parser.add_argument(
        "--save-map", action="store_true", help="Save map after navigation"
    )

    args = parser.parse_args()

    if args.interactive:
        await interactive_mode()
        return

    if args.goal_x is None or args.goal_y is None:
        print("[!] Error: --goal-x and --goal-y required (or use --interactive)")
        print("    Example: python goto_goal.py --goal-x 1000 --goal-y 2000")
        return

    # Create config
    config = NavigationConfig(
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        goal_yaw=math.radians(args.goal_yaw) if args.goal_yaw else None,
        use_lidar=not args.no_lidar,
        simulation_mode=args.simulation,
        save_map=args.save_map,
    )

    # Create navigator and run
    navigator = GotoGoalNavigator(config)

    try:
        navigator.connect_sensors()
        result = await navigator.navigate(args.goal_x, args.goal_y, config.goal_yaw)

        if args.save_map:
            navigator.save_map()

        # Print final result
        print(f"\nNavigation {'SUCCESS' if result['success'] else 'FAILED'}")
        print(f"State: {result['state']}")
        print(f"Final pose: {result['final_pose']}")

    except KeyboardInterrupt:
        print("\n[!] Navigation interrupted by user")
    finally:
        navigator.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
