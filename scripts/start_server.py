"""
HTTP Server for serving the Lidar SLAM visualization frontend
Also provides REST API for saving maps
"""

import json
import os
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from database import save_slam_map, get_slam_maps

FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "..", "templates")
PORT = 808


class LidarHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=FRONTEND_DIR, **kwargs)

    def do_GET(self):
        if self.path == "/":
            self.path = "/lidar_slam.html"
            return SimpleHTTPRequestHandler.do_GET(self)
        elif self.path == "/api/maps":
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            maps = get_slam_maps()
            self.wfile.write(json.dumps(maps).encode())
        else:
            return SimpleHTTPRequestHandler.do_GET(self)

    def do_POST(self):
        if self.path == "/api/map/save":
            content_length = int(self.headers["Content-Length"])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data)
                map_name = data.get("map_name", "unnamed")
                points = data.get("points", [])
                save_slam_map(map_name, points)

                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"success": True}).encode())
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()


def main():
    print(f"[*] Lidar SLAM HTTP Server")
    print(f"[*] Serving frontend at http://localhost:{PORT}")
    print(f"[*] WebSocket server should be running at ws://localhost:8766")

    server = HTTPServer(("0.0.0.0", PORT), LidarHTTPHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[*] Server stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
