"""
Robot Integration Server with Real Lidar
Connects autonomous mapping with real robot control and lidar scanning.
FIXED: Proper movement and position calibration
"""

import asyncio
import json
import math
import os
import sys
import time
import struct
import threading
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler
from threading import Thread
import urllib.request
import urllib.error

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
from database import get_connection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from occupancy_map import OccupancyMap
from path_planner import PathPlanner

try:
    import websockets
except ImportError:
    print("[!] websockets not installed. Run: poetry install")
    sys.exit(1)

try:
    import serial

    LIDAR_AVAILABLE = True
except ImportError:
    LIDAR_AVAILABLE = False
    print("[!] pyserial not installed. Lidar will use simulation.")

# ===== CONFIGURATION =====
LIDAR_PORT = "/dev/cu.usbserial-0001"
LIDAR_BAUD = 115200
WS_PORT = 8767
HTTP_PORT = 8082
ROBOT_API = "http://localhost:8000"
FORWARD_POWER = 2000
TURN_POWER = 1500
MOVE_DURATION = 0.3

# ===== GLOBAL STATE =====
connected_clients = set()
occupancy_map = OccupancyMap(resolution=50.0, width=400, height=400)
path_planner = PathPlanner(occupancy_map)
robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
simulation_mode = True
exploration_running = False
current_path = []
path_index = 0
should_exit = False
lidar_lock = threading.Lock()


# ===== LIDAR READER CLASS =====
class LidarReader:
    HEADER = bytes([0xAA, 0x55])

    def __init__(self, port=LIDAR_PORT, baudrate=LIDAR_BAUD):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False

    def connect(self):
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
            print(f"[+] Lidar connected: {self.port}")
            return True
        except Exception as e:
            print(f"[!] Lidar connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self):
        if self.serial and self.connected:
            self.serial.close()
            self.connected = False

    @staticmethod
    def calc_angle(raw_angle):
        return (raw_angle >> 1) / 64.0

    def read_packet(self):
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

    def parse_packet(self, data):
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

    def scan_360(self):
        if not self.connected:
            return None
        scan_points = []
        scan_complete = False
        while not scan_complete:
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
        return scan_points


# ===== SIMULATED ROBOT CLASS =====
class SimulatedRobot:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.room_config = self._create_room()
        self.map_name = "default"
        print(
            f"[*] SimRobot at ({self.x:.0f}, {self.y:.0f}), yaw={math.degrees(self.yaw):.1f}°"
        )

    def _create_room(self):
        return {
            "walls": [
                {"angle_start": 0, "angle_end": 45, "distance": 2500},
                {"angle_start": 45, "angle_end": 90, "distance": 3000},
                {"angle_start": 90, "angle_end": 135, "distance": 2500},
                {"angle_start": 135, "angle_end": 180, "distance": 3500},
                {"angle_start": 180, "angle_end": 225, "distance": 3000},
                {"angle_start": 225, "angle_end": 270, "distance": 2800},
                {"angle_start": 270, "angle_end": 315, "distance": 3200},
                {"angle_start": 315, "angle_end": 360, "distance": 2500},
            ],
            "obstacles": [
                {"angle": 30, "width": 15, "distance": 1200},
                {"angle": 90, "width": 20, "distance": 1500},
                {"angle": 180, "width": 25, "distance": 1800},
                {"angle": 270, "width": 15, "distance": 1400},
            ],
        }

    def load_map(self, map_name):
        """Load a map from JSON file"""
        maps_dir = os.path.join(os.path.dirname(__file__), "maps")
        map_file = os.path.join(maps_dir, f"{map_name}.json")

        if not os.path.exists(map_file):
            print(f"[!] Map not found: {map_file}")
            return False

        try:
            with open(map_file, "r") as f:
                self.room_config = json.load(f)
            self.map_name = map_name
            print(f"[*] Loaded map: {map_name}")
            return True
        except Exception as e:
            print(f"[!] Error loading map: {e}")
            return False

    @staticmethod
    def list_maps():
        """List available maps"""
        maps_dir = os.path.join(os.path.dirname(__file__), "maps")
        if not os.path.exists(maps_dir):
            return []
        maps = []
        for f in os.listdir(maps_dir):
            if f.endswith(".json"):
                try:
                    with open(os.path.join(maps_dir, f), "r") as fp:
                        data = json.load(fp)
                        maps.append(
                            {
                                "name": f.replace(".json", ""),
                                "description": data.get("description", ""),
                            }
                        )
                except:
                    pass
        return maps

    def scan(self):
        points = []
        for angle in range(0, 360, 2):
            wall_dist = 5000
            for wall in self.room_config["walls"]:
                if wall["angle_start"] <= angle < wall["angle_end"]:
                    wall_dist = wall["distance"]
                    break
            for obs in self.room_config["obstacles"]:
                angle_diff = abs(angle - obs["angle"])
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                if angle_diff < obs["width"]:
                    wall_dist = min(wall_dist, obs["distance"])
            noise = (hash(f"{angle}{time.time()}") % 40) - 20
            distance = max(100, wall_dist + noise)
            points.append({"angle": angle, "distance": int(distance)})
        return points

    def move_forward(self, distance_mm):
        self.x += distance_mm * math.cos(self.yaw)
        self.y += distance_mm * math.sin(self.yaw)
        return self.get_pose()

    def turn(self, angle_deg):
        self.yaw += math.radians(angle_deg)
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        return self.get_pose()

    def set_pose(self, x, y, yaw=None):
        self.x = float(x)
        self.y = float(y)
        if yaw is not None:
            self.yaw = float(yaw)
        return self.get_pose()

    def get_pose(self):
        return {"x": self.x, "y": self.y, "yaw": self.yaw}


# ===== INITIALIZE GLOBAL INSTANCES =====
lidar = LidarReader()
sim_robot = SimulatedRobot()


# ===== ROBOT CONTROL FUNCTIONS =====
def robot_api_call(endpoint, data=None, method="GET"):
    try:
        url = f"{ROBOT_API}{endpoint}"
        headers = {"Content-Type": "application/json"}
        if data:
            req = urllib.request.Request(
                url, data=json.dumps(data).encode(), headers=headers, method=method
            )
        else:
            req = urllib.request.Request(url, method=method)
        with urllib.request.urlopen(req, timeout=5) as response:
            return json.loads(response.read().decode())
    except Exception:
        return None


def check_robot_connection():
    global simulation_mode
    result = robot_api_call("/api/status")
    if result and result.get("connected") == "true":
        simulation_mode = False
        print("[+] Robot connected! Real robot mode.")
        return True
    simulation_mode = True
    print("[*] Robot not connected. Simulation mode.")
    return False


def check_lidar_connection():
    if lidar.connect():
        return True
    print("[*] Lidar not available. Simulated scans.")
    return False


async def send_robot_command(action, power=None, duration=None):
    global simulation_mode, robot_pose

    if simulation_mode:
        if action == "forward":
            dist = FORWARD_POWER * (duration or MOVE_DURATION) * 0.1
            sim_robot.move_forward(dist)
        elif action == "backward":
            dist = -FORWARD_POWER * (duration or MOVE_DURATION) * 0.1
            sim_robot.move_forward(dist)
        elif action == "left":
            angle = TURN_POWER * (duration or MOVE_DURATION) * 0.5
            sim_robot.turn(angle)
        elif action == "right":
            angle = -TURN_POWER * (duration or MOVE_DURATION) * 0.5
            sim_robot.turn(angle)
        robot_pose = sim_robot.get_pose()
        print(
            f"  [SimRobot] {action} -> ({robot_pose['x']:.0f}, {robot_pose['y']:.0f}), yaw={math.degrees(robot_pose['yaw']):.1f}°"
        )
        return True

    try:
        data = {"action": action}
        if power:
            data["power"] = power
        if duration:
            data["duration"] = duration
        robot_api_call("/api/move", data, "POST")
        return True
    except Exception as e:
        print(f"[!] Robot command failed: {e}")
        return False


# ===== WEBSOCKET BROADCAST =====
async def broadcast_data(data):
    if connected_clients:
        msg = json.dumps(data)
        await asyncio.gather(
            *[c.send(msg) for c in connected_clients], return_exceptions=True
        )


# ===== SCAN AND MAP UPDATE =====
async def perform_scan():
    global robot_pose
    if simulation_mode:
        robot_pose = sim_robot.get_pose()

    if lidar.connected:
        with lidar_lock:
            points = lidar.scan_360()
        if points:
            for p in points:
                angle_rad = math.radians(p["angle"])
                p["world_x"] = robot_pose["x"] + p["distance"] * math.cos(
                    angle_rad + robot_pose["yaw"]
                )
                p["world_y"] = robot_pose["y"] + p["distance"] * math.sin(
                    angle_rad + robot_pose["yaw"]
                )
            return points

    return sim_robot.scan()


async def update_map_from_scan():
    global occupancy_map, robot_pose
    scan_data = await perform_scan()
    if scan_data:
        occupancy_map.update_scan(
            robot_pose["x"], robot_pose["y"], robot_pose["yaw"], scan_data
        )
        scan_points = [
            {"angle": p["angle"], "distance": p["distance"]}
            for p in scan_data
            if p.get("distance", 0) > 0
        ][:180]
        await broadcast_data(
            {
                "type": "map_update",
                "map": occupancy_map.to_dict(),
                "robot_pose": robot_pose,
                "scan_source": "lidar" if lidar.connected else "simulation",
                "scan_points": scan_points,
            }
        )
    return scan_data


# ===== EXPLORATION LOOP =====
async def exploration_loop():
    global exploration_running, current_path, path_index, robot_pose

    print("[*] === EXPLORATION STARTED ===")
    scan_count = 0

    # Initial scans
    print("[*] Initial scans...")
    for i in range(3):
        if not exploration_running:
            break
        await update_map_from_scan()
        scan_count += 1
        await asyncio.sleep(0.1)

    print(f"[*] Done. {scan_count} scans.")

    while exploration_running and not should_exit:
        try:
            # Scan
            await update_map_from_scan()
            scan_count += 1

            # Find frontiers
            frontiers = occupancy_map.find_frontiers(5)
            print(f"[*] Scan {scan_count}: {len(frontiers)} frontiers")

            if not frontiers:
                for _ in range(5):
                    if not exploration_running:
                        break
                    await update_map_from_scan()
                    scan_count += 1
                    frontiers = occupancy_map.find_frontiers(5)
                    if frontiers:
                        break
                    await asyncio.sleep(0.1)

                if not frontiers:
                    print("[*] Exploration complete!")
                    await broadcast_data(
                        {"type": "exploration_completed", "total_scans": scan_count}
                    )
                    exploration_running = False
                    break

            # Select goal
            goal = path_planner.find_frontier_goal(
                frontiers, (robot_pose["x"], robot_pose["y"])
            )
            if not goal:
                await asyncio.sleep(0.3)
                continue

            print(f"[*] Goal: ({goal[0]:.0f}, {goal[1]:.0f})")

            # Plan path
            path = path_planner.plan_rrt((robot_pose["x"], robot_pose["y"]), goal)
            if not path or len(path) < 2:
                await asyncio.sleep(0.3)
                continue

            # Simplify path
            simple_path = [path[0]]
            for i in range(1, len(path)):
                dx = path[i][0] - simple_path[-1][0]
                dy = path[i][1] - simple_path[-1][1]
                if math.sqrt(dx * dx + dy * dy) > 200:
                    simple_path.append(path[i])
            simple_path.append(path[-1])
            current_path = simple_path

            print(f"[*] Path: {len(current_path)} waypoints")
            await broadcast_data(
                {"type": "path_planned", "path": current_path, "goal": goal}
            )

            # Follow path
            path_index = 1
            while path_index < len(current_path) and exploration_running:
                target_x, target_y = current_path[path_index]
                dx = target_x - robot_pose["x"]
                dy = target_y - robot_pose["y"]
                distance = math.sqrt(dx * dx + dy * dy)
                target_yaw = math.atan2(dy, dx)
                yaw_diff = target_yaw - robot_pose["yaw"]
                while yaw_diff > math.pi:
                    yaw_diff -= 2 * math.pi
                while yaw_diff < -math.pi:
                    yaw_diff += 2 * math.pi

                # Turn
                if abs(yaw_diff) > 0.15:
                    turn_deg = math.degrees(yaw_diff)
                    print(f"  [Turn] {turn_deg:.1f}°")
                    await send_robot_command(
                        "left" if turn_deg > 0 else "right", TURN_POWER, 0.2
                    )
                    await asyncio.sleep(0.3)
                # Move forward
                elif distance > 150:
                    print(f"  [Move] forward")
                    await send_robot_command("forward", FORWARD_POWER, 0.3)
                    await asyncio.sleep(0.3)
                else:
                    path_index += 1

                await broadcast_data({"type": "robot_moved", "pose": robot_pose})
                await asyncio.sleep(0.1)

            await asyncio.sleep(0.2)

        except Exception as e:
            print(f"[!] Error: {e}")
            import traceback

            traceback.print_exc()
            await asyncio.sleep(0.5)

    print(f"[*] === EXPLORATION DONE === Scans: {scan_count}")
    exploration_running = False


# ===== WEBSOCKET HANDLER =====
async def websocket_handler(websocket, path=None):
    global \
        exploration_running, \
        simulation_mode, \
        occupancy_map, \
        lidar, \
        robot_pose, \
        sim_robot, \
        path_planner, \
        current_path

    connected_clients.add(websocket)
    print(f"[+] Client connected. Total: {len(connected_clients)}")

    try:
        await websocket.send(
            json.dumps(
                {
                    "type": "connected",
                    "simulation_mode": simulation_mode,
                    "lidar_connected": lidar.connected,
                    "robot_pose": robot_pose,
                }
            )
        )

        async for message in websocket:
            try:
                data = json.loads(message)
                cmd = data.get("command")
                print(f"[*] Cmd: {cmd}")

                if cmd == "check_robot":
                    ok = check_robot_connection()
                    await websocket.send(
                        json.dumps(
                            {
                                "type": "robot_status",
                                "connected": ok,
                                "simulation_mode": simulation_mode,
                                "lidar_connected": lidar.connected,
                            }
                        )
                    )

                elif cmd == "check_lidar":
                    ok = check_lidar_connection()
                    await websocket.send(
                        json.dumps({"type": "lidar_status", "connected": ok})
                    )

                elif cmd == "start_exploration":
                    if not exploration_running:
                        exploration_running = True
                        asyncio.create_task(exploration_loop())
                        await websocket.send(
                            json.dumps({"type": "exploration_started"})
                        )

                elif cmd == "stop_exploration":
                    exploration_running = False
                    await websocket.send(json.dumps({"type": "exploration_stopped"}))

                elif cmd == "reset":
                    global occupancy_map, current_path
                    exploration_running = False
                    occupancy_map = OccupancyMap(resolution=50.0, width=400, height=400)
                    path_planner = PathPlanner(occupancy_map)
                    sim_robot.set_pose(0, 0, 0)
                    robot_pose = sim_robot.get_pose()
                    current_path = []
                    await broadcast_data({"type": "reset_complete", "pose": robot_pose})

                elif cmd == "scan":
                    await update_map_from_scan()
                    await websocket.send(json.dumps({"type": "scan_complete"}))

                elif cmd == "manual_move":
                    action = data.get("action", "stop")
                    power = data.get(
                        "power",
                        FORWARD_POWER
                        if action in ["forward", "backward"]
                        else TURN_POWER,
                    )
                    duration = data.get("duration", 0.5)
                    await send_robot_command(action, power, duration)
                    await broadcast_data({"type": "robot_moved", "pose": robot_pose})

                elif cmd == "calibrate_position":
                    x = data.get("x", 0)
                    y = data.get("y", 0)
                    yaw_deg = data.get("yaw", 0)
                    if simulation_mode:
                        sim_robot.set_pose(x, y, math.radians(yaw_deg))
                        robot_pose = sim_robot.get_pose()
                    else:
                        robot_pose = {"x": x, "y": y, "yaw": math.radians(yaw_deg)}
                    await broadcast_data(
                        {"type": "position_calibrated", "pose": robot_pose}
                    )

                elif cmd == "get_position":
                    if simulation_mode:
                        robot_pose = sim_robot.get_pose()
                    await websocket.send(
                        json.dumps({"type": "position_update", "pose": robot_pose})
                    )

                elif cmd == "save_map":
                    name = data.get("map_name", f"map_{int(time.time())}")
                    conn = get_connection()
                    c = conn.cursor()
                    c.execute(
                        "INSERT INTO slam_map (map_name, points_json) VALUES (?, ?)",
                        (name, json.dumps(occupancy_map.to_dict())),
                    )
                    conn.commit()
                    conn.close()
                    await websocket.send(
                        json.dumps({"type": "map_saved", "name": name})
                    )

                elif cmd == "list_maps":
                    maps = SimulatedRobot.list_maps()
                    await websocket.send(json.dumps({"type": "map_list", "maps": maps}))

                elif cmd == "load_map":
                    map_name = data.get("map_name", "simple_room")
                    if sim_robot.load_map(map_name):
                        occupancy_map = OccupancyMap(
                            resolution=50.0, width=400, height=400
                        )
                        path_planner = PathPlanner(occupancy_map)
                        sim_robot.set_pose(0, 0, 0)
                        robot_pose = sim_robot.get_pose()
                        current_path = []
                        print(f"[*] Map '{map_name}' loaded and reset")
                        await update_map_from_scan()
                        await websocket.send(
                            json.dumps(
                                {
                                    "type": "map_loaded",
                                    "map_name": map_name,
                                    "pose": robot_pose,
                                }
                            )
                        )
                    else:
                        await websocket.send(
                            json.dumps(
                                {
                                    "type": "error",
                                    "message": f"Map not found: {map_name}",
                                }
                            )
                        )

            except json.JSONDecodeError:
                await websocket.send(
                    json.dumps({"type": "error", "message": "Invalid JSON"})
                )

    except Exception as e:
        print(f"[!] WS Error: {e}")
    finally:
        connected_clients.discard(websocket)
        print(f"[-] Client disconnected. Total: {len(connected_clients)}")


# ===== HTTP SERVER =====
def run_http_server():
    frontend_dir = os.path.dirname(os.path.abspath(__file__))

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=frontend_dir, **kwargs)

        def end_headers(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            super().end_headers()

        def do_GET(self):
            if self.path == "/":
                self.path = "/index.html"
            return super().do_GET()

    server = HTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[+] HTTP: http://localhost:{HTTP_PORT}")
    server.serve_forever()


def init_db():
    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "CREATE TABLE IF NOT EXISTS slam_map (id INTEGER PRIMARY KEY AUTOINCREMENT, timestamp DATETIME DEFAULT CURRENT_TIMESTAMP, map_name TEXT, points_json TEXT)"
    )
    conn.commit()
    conn.close()


async def main():
    global should_exit
    print("\n" + "=" * 60)
    print("  Robot Autonomous Mapping Server v2")
    print("=" * 60)
    print(f"  HTTP: http://localhost:{HTTP_PORT}")
    print(f"  WS:   ws://localhost:{WS_PORT}")
    print("=" * 60)

    init_db()
    check_robot_connection()
    check_lidar_connection()

    print("[*] Ctrl+C to stop")
    print("[*] Commands: Forward/Back=2000, Left/Right=1500")
    print("=" * 60 + "\n")

    Thread(target=run_http_server, daemon=True).start()

    try:
        async with websockets.serve(websocket_handler, "0.0.0.0", WS_PORT):
            await asyncio.Future()
    except KeyboardInterrupt:
        print("\n[*] Shutting down...")
        should_exit = True
        lidar.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[*] Server stopped.")
