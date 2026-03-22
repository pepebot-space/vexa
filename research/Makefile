.PHONY: help install run-lidar-basic run-slam-viz run-slam-ws run-slam-http run-auto stop clean shell dev

help:
	@echo "Available commands:"
	@echo "  make install           - Install dependencies with Poetry"
	@echo "  make shell             - Activate Poetry virtual environment shell"
	@echo ""
	@echo "Lidar Basic:"
	@echo "  make run-lidar-basic   - Run basic lidar reader"
	@echo ""
	@echo "SLAM Visualization:"
	@echo "  make run-slam-viz      - Run both WebSocket and HTTP servers"
	@echo "  make run-slam-ws       - Run WebSocket server only (port 8766)"
	@echo "  make run-slam-http     - Run HTTP server only (port 808)"
	@echo ""
	@echo "Autonomous Mapping:"
	@echo "  make run-auto          - Run combined server (port 8767 + 8082)"
	@echo ""
	@echo "Utility:"
	@echo "  make stop              - Stop all running servers"
	@echo "  make clean             - Clean Python cache"

install:
	poetry install

shell:
	poetry shell

run-lidar-basic:
	poetry run python lidar_basic/lidar_reader.py

run-slam-ws:
	poetry run python slam_visualization/lidar_ws_server.py

run-slam-http:
	poetry run python slam_visualization/start_server.py

run-slam-viz:
	@echo "Starting SLAM Visualization servers..."
	@poetry run python slam_visualization/lidar_ws_server.py &
	@poetry run python slam_visualization/start_server.py &
	@echo "SLAM WebSocket: ws://localhost:8766"
	@echo "SLAM HTTP: http://localhost:808"

run-auto:
	@echo "Starting Autonomous Mapping server..."
	@poetry run python autonomous_mapping/server.py

stop:
	@pkill -f "lidar_ws_server.py" 2>/dev/null || true
	@pkill -f "start_server.py" 2>/dev/null || true
	@pkill -f "websocket_server.py" 2>/dev/null || true
	@pkill -f "http_server.py" 2>/dev/null || true
	@pkill -f "lidar_reader.py" 2>/dev/null || true
	@pkill -f "autonomous_mapping/server.py" 2>/dev/null || true
	@echo "All servers stopped"

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@echo "Cleaned Python cache"

dev:
	@echo "Development mode - installing with Poetry..."
	poetry install
	@echo ""
	@echo "Run 'make shell' to activate virtual environment"
	@echo "Or use 'poetry run python <script.py>' to run directly"