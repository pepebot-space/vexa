import sqlite3
import datetime
import os

DB_PATH = os.path.join(os.path.dirname(__file__), "minirobot.db")


def get_connection():
    # Use check_same_thread=False since we will use asyncio.to_thread in FastAPI
    conn = sqlite3.connect(DB_PATH, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    conn = get_connection()
    c = conn.cursor()
    c.execute("""
        CREATE TABLE IF NOT EXISTS robot_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            level TEXT,
            component TEXT,
            message TEXT
        )
    """)
    c.execute("""
        CREATE TABLE IF NOT EXISTS battery_history (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            battery_percent INTEGER,
            voltage REAL,
            raw_deci_volt INTEGER
        )
    """)
    c.execute("""
        CREATE TABLE IF NOT EXISTS settings (
            key TEXT PRIMARY KEY,
            value TEXT
        )
    """)
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


def add_log(level: str, component: str, message: str):
    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO robot_logs (level, component, message) VALUES (?, ?, ?)",
        (level, component, message),
    )
    conn.commit()
    conn.close()


def get_logs(limit: int = 50):
    conn = get_connection()
    c = conn.cursor()
    c.execute("SELECT * FROM robot_logs ORDER BY id DESC LIMIT ?", (limit,))
    rows = c.fetchall()
    conn.close()
    return [dict(row) for row in rows]


def add_battery_record(percent: int, voltage: float, raw_deci_volt: int):
    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO battery_history (battery_percent, voltage, raw_deci_volt) VALUES (?, ?, ?)",
        (percent, voltage, raw_deci_volt),
    )
    conn.commit()
    conn.close()


def get_battery_history(limit: int = 24):
    conn = get_connection()
    c = conn.cursor()
    c.execute("SELECT * FROM battery_history ORDER BY id DESC LIMIT ?", (limit,))
    rows = c.fetchall()
    conn.close()
    # Reverse to return chronological order
    return [dict(row) for row in reversed(rows)]


def set_setting(key: str, value: str):
    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO settings (key, value) VALUES (?, ?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",
        (key, str(value)),
    )
    conn.commit()
    conn.close()


def get_setting(key: str, default: str = None) -> str:
    conn = get_connection()
    c = conn.cursor()
    c.execute("SELECT value FROM settings WHERE key = ?", (key,))
    row = c.fetchone()
    conn.close()
    return row["value"] if row else default


def save_lidar_scan(scan_id: int, points: list):
    import json

    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO lidar_scans (scan_id, point_count, points_json) VALUES (?, ?, ?)",
        (scan_id, len(points), json.dumps(points)),
    )
    conn.commit()
    conn.close()


def get_lidar_scans(limit: int = 100):
    conn = get_connection()
    c = conn.cursor()
    c.execute("SELECT * FROM lidar_scans ORDER BY id DESC LIMIT ?", (limit,))
    rows = c.fetchall()
    conn.close()
    return [dict(row) for row in rows]


def save_slam_map(map_name: str, points: list):
    import json

    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO slam_map (map_name, points_json) VALUES (?, ?)",
        (map_name, json.dumps(points)),
    )
    conn.commit()
    conn.close()


def get_slam_maps():
    conn = get_connection()
    c = conn.cursor()
    c.execute("SELECT * FROM slam_map ORDER BY id DESC")
    rows = c.fetchall()
    conn.close()
    return [dict(row) for row in rows]


# Initialize DB when module is imported
init_db()
