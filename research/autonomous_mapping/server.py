"""
Combined Server for Autonomous Room Mapping
Single process that runs both WebSocket and HTTP servers.
"""

import asyncio
import json
import math
import os
import sys
import time
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler
from threading import Thread

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
from database import get_connection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from auto_explorer import AutoExplorer, SimulatedRobot, ExplorerState
from occupancy_map import OccupancyMap

try:
    import websockets
except ImportError:
    print("[!] websockets not installed. Run: poetry install")
    sys.exit(1)

WS_PORT = 8767
HTTP_PORT = 8082

connected_clients = set()
explorer = AutoExplorer(map_resolution=50.0, map_size=(400, 400))
sim_robot = SimulatedRobot()
simulation_running = False
should_exit = False


async def broadcast_data(data: dict):
    if connected_clients:
        message = json.dumps(data)
        await asyncio.gather(
            *[client.send(message) for client in connected_clients],
            return_exceptions=True,
        )


async def map_update_broadcast():
    await broadcast_data(
        {
            "type": "map_update",
            "map": explorer.occupancy_map.to_dict(),
            "robot_pose": {
                "x": sim_robot.pose.x,
                "y": sim_robot.pose.y,
                "yaw": sim_robot.pose.yaw,
            },
        }
    )


async def simulation_loop():
    global simulation_running, explorer, sim_robot

    while simulation_running and not should_exit:
        try:
            if explorer.state == ExplorerState.SCANNING:
                scan_data = sim_robot.simulate_scan()
                explorer.update_robot_pose(
                    sim_robot.pose.x, sim_robot.pose.y, sim_robot.pose.yaw
                )
                explorer.update_map(scan_data)
                await broadcast_data(
                    {
                        "type": "scan_data",
                        "points": scan_data,
                        "pose": {
                            "x": sim_robot.pose.x,
                            "y": sim_robot.pose.y,
                            "yaw": sim_robot.pose.yaw,
                        },
                    }
                )
                explorer.state = ExplorerState.PLANNING

            elif explorer.state == ExplorerState.PLANNING:
                frontiers = explorer.occupancy_map.find_frontiers(10)

                if not frontiers:
                    explorer.state = ExplorerState.COMPLETED
                    await broadcast_data(
                        {
                            "type": "exploration_completed",
                            "stats": explorer._get_stats_dict(),
                        }
                    )
                    simulation_running = False
                else:
                    goal = explorer.path_planner.find_frontier_goal(
                        frontiers, (sim_robot.pose.x, sim_robot.pose.y)
                    )

                    if goal:
                        path = explorer.path_planner.plan_rrt(
                            (sim_robot.pose.x, sim_robot.pose.y), goal
                        )

                        if path:
                            explorer.current_path = path
                            explorer.current_path_index = 1
                            explorer.stats.frontiers_visited += 1
                            explorer.state = ExplorerState.MOVING

                            await broadcast_data(
                                {"type": "path_planned", "path": path, "goal": goal}
                            )
                        else:
                            explorer.state = ExplorerState.SCANNING
                    else:
                        explorer.state = ExplorerState.SCANNING

            elif explorer.state == ExplorerState.MOVING:
                if not explorer.current_path or explorer.current_path_index >= len(
                    explorer.current_path
                ):
                    explorer.state = ExplorerState.SCANNING
                else:
                    target_x, target_y = explorer.current_path[
                        explorer.current_path_index
                    ]

                    dx = target_x - sim_robot.pose.x
                    dy = target_y - sim_robot.pose.y
                    distance = (dx**2 + dy**2) ** 0.5

                    if distance < 50:
                        explorer.current_path_index += 1
                    else:
                        target_yaw = explorer.path_planner._normalize_angle(
                            explorer.path_planner._normalize_angle(math.atan2(dy, dx))
                            - sim_robot.pose.yaw
                        )
                        if abs(target_yaw) > 0.1:
                            sim_robot.move(0, 30 if target_yaw > 0 else -30, 0.1)
                        else:
                            speed = min(100, distance * 0.5)
                            sim_robot.move(speed, 0, 0.1)

                        await broadcast_data(
                            {
                                "type": "robot_moved",
                                "pose": {
                                    "x": sim_robot.pose.x,
                                    "y": sim_robot.pose.y,
                                    "yaw": sim_robot.pose.yaw,
                                },
                            }
                        )

                    await map_update_broadcast()

            elif explorer.state == ExplorerState.COMPLETED:
                simulation_running = False
                break

            await asyncio.sleep(0.05)

        except Exception as e:
            print(f"[!] Error in simulation: {e}")
            await asyncio.sleep(0.1)


async def websocket_handler(websocket, path=None):
    global simulation_running, explorer, sim_robot
    connected_clients.add(websocket)
    print(f"[+] Client connected. Total: {len(connected_clients)}")

    try:
        await websocket.send(
            json.dumps(
                {
                    "type": "connected",
                    "message": "Connected to Autonomous Mapping Server",
                }
            )
        )

        await websocket.send(
            json.dumps(
                {
                    "type": "map_update",
                    "map": explorer.occupancy_map.to_dict(),
                    "robot_pose": {
                        "x": sim_robot.pose.x,
                        "y": sim_robot.pose.y,
                        "yaw": sim_robot.pose.yaw,
                    },
                }
            )
        )

        async for message in websocket:
            try:
                data = json.loads(message)
                command = data.get("command")

                if command == "start_exploration":
                    if not simulation_running:
                        simulation_running = True
                        asyncio.create_task(simulation_loop())
                        await websocket.send(
                            json.dumps(
                                {
                                    "type": "exploration_started",
                                    "message": "Autonomous exploration started",
                                }
                            )
                        )

                elif command == "stop_exploration":
                    simulation_running = False
                    explorer.state = ExplorerState.IDLE
                    await websocket.send(
                        json.dumps(
                            {
                                "type": "exploration_stopped",
                                "message": "Exploration stopped",
                            }
                        )
                    )

                elif command == "reset":
                    simulation_running = False
                    explorer = AutoExplorer(map_resolution=50.0, map_size=(400, 400))
                    sim_robot = SimulatedRobot()
                    await map_update_broadcast()
                    await websocket.send(
                        json.dumps(
                            {"type": "reset_complete", "message": "Map and robot reset"}
                        )
                    )

                elif command == "save_map":
                    map_name = data.get("map_name", f"exploration_{int(time.time())}")
                    conn = get_connection()
                    c = conn.cursor()
                    c.execute(
                        "INSERT INTO slam_map (map_name, points_json) VALUES (?, ?)",
                        (map_name, json.dumps(explorer.occupancy_map.to_dict())),
                    )
                    conn.commit()
                    conn.close()
                    await websocket.send(
                        json.dumps({"type": "map_saved", "map_name": map_name})
                    )

                elif command == "get_map":
                    await websocket.send(
                        json.dumps(
                            {
                                "type": "map_update",
                                "map": explorer.occupancy_map.to_dict(),
                            }
                        )
                    )

                elif command == "set_robot_pose":
                    x = data.get("x", 0)
                    y = data.get("y", 0)
                    yaw = data.get("yaw", 0)
                    sim_robot.set_pose(x, y, yaw)
                    await websocket.send(
                        json.dumps(
                            {
                                "type": "robot_pose_set",
                                "pose": {"x": x, "y": y, "yaw": yaw},
                            }
                        )
                    )

                elif command == "manual_move":
                    linear_x = data.get("linear_x", 0)
                    angular_z = data.get("angular_z", 0)
                    sim_robot.move(linear_x, angular_z, 0.1)
                    await broadcast_data(
                        {
                            "type": "robot_moved",
                            "pose": {
                                "x": sim_robot.pose.x,
                                "y": sim_robot.pose.y,
                                "yaw": sim_robot.pose.yaw,
                            },
                        }
                    )

                elif command == "manual_scan":
                    scan_data = sim_robot.simulate_scan()
                    explorer.update_robot_pose(
                        sim_robot.pose.x, sim_robot.pose.y, sim_robot.pose.yaw
                    )
                    explorer.update_map(scan_data)
                    await map_update_broadcast()
                    await websocket.send(
                        json.dumps({"type": "scan_complete", "points": len(scan_data)})
                    )

            except json.JSONDecodeError:
                await websocket.send(
                    json.dumps({"type": "error", "message": "Invalid JSON"})
                )

    except Exception as e:
        print(f"[!] WebSocket error: {e}")

    finally:
        connected_clients.discard(websocket)
        print(f"[-] Client disconnected. Total: {len(connected_clients)}")


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
    print(f"[+] HTTP server: http://localhost:{HTTP_PORT}")
    server.serve_forever()


def init_db():
    conn = get_connection()
    c = conn.cursor()
    c.execute("""
        CREATE TABLE IF NOT EXISTS slam_map (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            map_name TEXT,
            points_json TEXT
        )
    """)
    conn.commit()
    conn.close()


async def run_websocket_server():
    print(f"[+] WebSocket server: ws://localhost:{WS_PORT}")
    async with websockets.serve(websocket_handler, "0.0.0.0", WS_PORT):
        await asyncio.Future()


async def main():
    global should_exit

    print("\n" + "=" * 50)
    print("  Autonomous Room Mapping Server")
    print("=" * 50)

    init_db()

    http_thread = Thread(target=run_http_server, daemon=True)
    http_thread.start()

    print(f"[*] Open http://localhost:{HTTP_PORT} to view visualization")
    print("[*] Press Ctrl+C to stop\n")

    try:
        await run_websocket_server()
    except KeyboardInterrupt:
        print("\n[*] Shutting down...")
        should_exit = True


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[*] Server stopped.")
