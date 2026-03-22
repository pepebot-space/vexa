"""
HTTP Server for Autonomous Mapping Frontend
"""

import os
from http.server import HTTPServer, SimpleHTTPRequestHandler

PORT = 8082
FRONTEND_DIR = os.path.dirname(os.path.abspath(__file__))


class CORSRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=FRONTEND_DIR, **kwargs)

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def do_GET(self):
        if self.path == "/":
            self.path = "/index.html"
        return super().do_GET()


def main():
    print(f"[*] Autonomous Mapping HTTP Server")
    print(f"[*] Serving frontend at http://localhost:{PORT}")
    print(f"[*] WebSocket server should be running at ws://localhost:8767")

    server = HTTPServer(("0.0.0.0", PORT), CORSRequestHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[*] Server stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
