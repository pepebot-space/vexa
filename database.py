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
    c.execute('''
        CREATE TABLE IF NOT EXISTS robot_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            level TEXT,
            component TEXT,
            message TEXT
        )
    ''')
    c.execute('''
        CREATE TABLE IF NOT EXISTS battery_history (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            battery_percent INTEGER,
            voltage REAL,
            raw_deci_volt INTEGER
        )
    ''')
    conn.commit()
    conn.close()

def add_log(level: str, component: str, message: str):
    conn = get_connection()
    c = conn.cursor()
    c.execute(
        "INSERT INTO robot_logs (level, component, message) VALUES (?, ?, ?)",
        (level, component, message)
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
        (percent, voltage, raw_deci_volt)
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

# Initialize DB when module is imported
init_db()
