"""
YDLIDAR X2 WebSocket Server for SLAM Visualization
Broadcasts lidar scan data to connected WebSocket clients and saves to database.
"""

import asyncio
import json
import os
import struct
import sys
import time
from datetime import datetime
import websockets
import serial

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
from database import get_connection

SERIAL_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 115200
HEADER = bytes([0xAA, 0x55])
WS_PORT = 8766

connected_clients = set()


def calc_angle(raw_angle):
    return (raw_angle >> 1) / 64.0


def parse_packet(data):
    if len(data) < 10:
        return None

    ct = data[0]
    lsn = data[1]
    fsa = struct.unpack_from("<H", data, 2)[0]
    lsa = struct.unpack_from("<H", data, 4)[0]

    start_angle = calc_angle(fsa)
    end_angle = calc_angle(lsa)

    if end_angle < start_angle:
        angle_diff = (360.0 - start_angle) + end_angle
    else:
        angle_diff = end_angle - start_angle

    expected_data_len = lsn * 2
    sample_data = data[8:]

    if len(sample_data) < expected_data_len:
        return None

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


def read_packet(ser):
    while True:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == 0xAA:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x55:
                break

    header_data = ser.read(8)
    if len(header_data) < 8:
        return None

    lsn = header_data[1]
    data_len = lsn * 2
    sample_data = ser.read(data_len)

    if len(sample_data) < data_len:
        return None

    return header_data + sample_data


def init_lidar_table():
    conn = get_connection()
    c = conn.cursor()
    c.execute("""
        CREATE TABLE IF NOT EXISTS lidar_scans (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            scan_id INTEGER,
            point_count INTEGER,
            points_json TEXT
        )
    """)
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


def save_scan_to_db(scan_id, points):
    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO lidar_scans (scan_id, point_count, points_json) VALUES (?, ?, ?)",
        (scan_id, len(points), json.dumps(points)),
    )
    conn.commit()
    conn.close()


def save_map_to_db(map_name, points):
    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO slam_map (map_name, points_json) VALUES (?, ?)",
        (map_name, json.dumps(points)),
    )
    conn.commit()
    conn.close()


async def broadcast_data(data):
    if connected_clients:
        message = json.dumps(data)
        await asyncio.gather(
            *[client.send(message) for client in connected_clients],
            return_exceptions=True,
        )


async def websocket_handler(websocket, path=None):
    connected_clients.add(websocket)
    print(f"[+] Client connected. Total: {len(connected_clients)}")
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        print(f"[-] Client disconnected. Total: {len(connected_clients)}")


async def lidar_reader(ser):
    scan_points = []
    scan_count = 0

    while True:
        try:
            raw = await asyncio.to_thread(read_packet, ser)
            if raw is None:
                continue

            result = parse_packet(raw)
            if result is None:
                continue

            points = result["points"]
            is_start = result["is_start"]

            if is_start and scan_points:
                scan_count += 1
                full_scan = scan_points.copy()
                save_scan_to_db(scan_count, full_scan)

                await broadcast_data(
                    {
                        "type": "scan_complete",
                        "scan_id": scan_count,
                        "point_count": len(full_scan),
                        "points": full_scan,
                        "timestamp": datetime.now().isoformat(),
                    }
                )
                scan_points = []

            scan_points.extend(points)

            await broadcast_data(
                {"type": "scan_progress", "points": points, "is_start": is_start}
            )

        except Exception as e:
            print(f"[!] Error reading lidar: {e}")
            await asyncio.sleep(0.1)


async def simulate_lidar():
    import math

    scan_count = 0
    room_walls = [
        {"angle_start": 0, "angle_end": 45, "base_dist": 2500},
        {"angle_start": 45, "angle_end": 90, "base_dist": 3000},
        {"angle_start": 90, "angle_end": 135, "base_dist": 2200},
        {"angle_start": 135, "angle_end": 180, "base_dist": 2800},
        {"angle_start": 180, "angle_end": 225, "base_dist": 2600},
        {"angle_start": 225, "angle_end": 270, "base_dist": 2400},
        {"angle_start": 270, "angle_end": 315, "base_dist": 3100},
        {"angle_start": 315, "angle_end": 360, "base_dist": 2700},
    ]

    obstacles = [
        {"angle": 30, "width": 15, "dist": 1500},
        {"angle": 120, "width": 20, "dist": 1200},
        {"angle": 200, "width": 25, "dist": 1800},
        {"angle": 280, "width": 10, "dist": 1000},
        {"angle": 350, "width": 12, "dist": 1400},
    ]

    while True:
        scan_points = []
        for angle_offset in range(0, 360, 2):
            angle = angle_offset

            wall_dist = 4000
            for wall in room_walls:
                if wall["angle_start"] <= angle < wall["angle_end"]:
                    wall_dist = wall["base_dist"]
                    break

            obs_dist = wall_dist
            for obs in obstacles:
                angle_diff = abs(angle - obs["angle"])
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                if angle_diff < obs["width"]:
                    obs_dist = obs["dist"]
                    break

            distance = min(wall_dist, obs_dist)
            noise = (hash(str(angle) + str(time.time())) % 60) - 30
            distance = max(100, min(4000, distance + noise))

            scan_points.append({"angle": angle, "distance": int(distance)})

            await broadcast_data(
                {
                    "type": "scan_progress",
                    "points": [{"angle": angle, "distance": int(distance)}],
                    "is_start": angle_offset == 0,
                }
            )
            await asyncio.sleep(0.01)

        scan_count += 1
        save_scan_to_db(scan_count, scan_points)

        await broadcast_data(
            {
                "type": "scan_complete",
                "scan_id": scan_count,
                "point_count": len(scan_points),
                "points": scan_points,
                "timestamp": datetime.now().isoformat(),
            }
        )


async def main():
    init_lidar_table()
    print(f"[*] YDLIDAR WebSocket Server")
    print(f"[*] WebSocket Port: {WS_PORT}")
    print(f"[*] Serial Port: {SERIAL_PORT}")

    ser = None
    use_simulation = False

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
        print(f"[+] Serial port connected!")
    except serial.SerialException as e:
        print(f"[!] Cannot open serial port: {e}")
        print(f"[*] Running in simulation mode...")
        use_simulation = True

    async with websockets.serve(websocket_handler, "0.0.0.0", WS_PORT):
        print(f"[+] WebSocket server started on ws://0.0.0.0:{WS_PORT}")
        print(f"[*] Open http://localhost:808 to view visualization")

        if use_simulation:
            await simulate_lidar()
        else:
            await lidar_reader(ser)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[*] Server stopped.")
