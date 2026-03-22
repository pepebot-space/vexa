# Robot Research - Autonomous Lidar SLAM

Collection of research projects for autonomous room mapping using LiDAR sensors.

## Directory Structure

```
research/
├── pyproject.toml            # Poetry configuration
├── Makefile                  # Build & run commands
├── README.md                 # This file
│
├── lidar_basic/              # Basic LiDAR reader
│   └── lidar_reader.py       # Serial communication with YDLIDAR X2
│
├── slam_visualization/       # Real-time SLAM visualization
│   ├── lidar_ws_server.py    # WebSocket server for real-time data
│   ├── start_server.py       # HTTP server for frontend
│   └── lidar_slam.html       # Three.js visualization
│
└── autonomous_mapping/       # Autonomous room exploration
    ├── occupancy_map.py      # Occupancy grid mapping
    ├── path_planner.py       # RRT path planning
    ├── auto_explorer.py      # Frontier-based exploration
    ├── websocket_server.py   # Real-time visualization
    ├── http_server.py        # Frontend server
    └── index.html           # Web UI
```

## Quick Start with Poetry

```bash
cd research

# Install dependencies
make install
# or: poetry install

# Activate virtual environment shell
make shell
# or: poetry shell

# Run autonomous mapping
make run-auto

# Run SLAM visualization
make run-slam-viz

# Stop all servers
make stop
```

## All Make Commands

```bash
make help              # Showall commands
make install           # Install dependencies with Poetry
make shell             # Activate Poetry virtualenv

# Lidar Basic
make run-lidar-basic   # Run basic lidar reader

# SLAM Visualization
make run-slam-viz      # Run both servers
make run-slam-ws       # WebSocket server only (port 8766)
make run-slam-http     # HTTP server only (port 808)

# Autonomous Mapping
make run-auto          # Run both servers
make run-auto-ws       # WebSocket server only (port 8767)
make run-auto-http     # HTTP server only (port 8081)

# Utility
make stop              # Stop all servers
make clean             # Clean Python cache
```

## Project Details

### 1. Lidar Basic (`lidar_basic/`)

Basic LiDAR reader for YDLIDAR X2 360° scanner.

**Features:**
- Serial communication at 115200 baud
- Parse YDLIDAR packet protocol
- ASCII visualization of scan data

**Run:**
```bash
make run-lidar-basic
```

---

### 2. SLAM Visualization (`slam_visualization/`)

Real-time LiDAR data visualization with WebSocket.

**Features:**
- Real-time 360° scan visualization
- 3D Three.js rendering
- 2D top-down view
- Database storage for scans
- Simulated mode (no hardware required)

**Run:**
```bash
make run-slam-viz
# WebSocket: ws://localhost:8766
# HTTP: http://localhost:808
```

**API:**
| Command | Description |
|---------|-------------|
| Connect | WebSocket: `ws://localhost:8766` |
| Save Map | POST `/api/map/save` |

---

### 3. Autonomous Mapping (`autonomous_mapping/`)

**Frontier-based autonomous exploration with RRT path planning.**

Paper Reference: [RELAX: Reinforcement Learning Enabled 2D-LiDAR Autonomous System](https://arxiv.org/html/2309.08095v2)

**Architecture:**

```
┌─────────────────────────────────────────────────────────────┐
│                    Autonomous Mapping System                 │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  LiDAR Scan  │→│    Map       │→│   Frontier      │  │
│  │              │  │  Constructor │  │   Detection     │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
│                            ↓                                 │
│                      ┌──────────────┐                        │
│                      │    Path      │                        │
│                      │   Planner    │                        │
│                      │   (RRT/RRT*) │                        │
│                      └──────────────┘                        │
│                            ↓                                 │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Navigation Controller                     │   │
│  │  • Move to waypoint                                    │   │
│  │  • Obstacle avoidance                                  │   │
│  │  • Scan at waypoints                                   │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

**Components:**

#### Occupancy Map (`occupancy_map.py`)
- Bayes log-odds update for occupancy probability
- Ray casting for free space marking
- Frontier detection (known/unknown boundaries)
- Grid resolution: 50mm per cell

#### Path Planner (`path_planner.py`)
- RRT (Rapidly Exploring Random Tree)
- RRT* for optimized paths
- Collision checking
- Path smoothing

#### Auto Explorer (`auto_explorer.py`)
- State machine: IDLE → SCANNING → PLANNING → MOVING → COMPLETED
- Frontier-based goal selection
- Exploration statistics tracking

**Run:**
```bash
make run-auto
# WebSocket: ws://localhost:8767
# HTTP: http://localhost:8082
```

**WebSocket Commands:**

| Command | Parameters | Description |
|---------|------------|-------------|
| `start_exploration` | - | Start autonomous exploration |
| `stop_exploration` | - | Stop exploration |
| `reset` | - | Reset map and robot position |
| `manual_scan` | - | Perform single scan |
| `save_map` | `map_name` | Save map to database |
| `manual_move` | `linear_x`, `angular_z` | Manual robot control |
| `set_robot_pose` | `x`, `y`, `yaw` | Set robot position |

---

## Algorithms

### Occupancy Grid Mapping

Based on Bayes' theorem with log-odds representation:

```
P(occupied | measurement) = 1 / (1 + exp(-log_odds))

log_odds_new = log_odds_old + log(P(measurement | occupied) / P(measurement | free))
```

### RRT Path Planning

```
1. Initialize tree with start node
2. For N iterations:
   a. Sample random point (or goal with bias)
   b. Find nearest node in tree
   c. Steer toward sample (limited step size)
   d. If collision-free, add new node
   e. If near goal, extract path
3. Smooth path using shortcutting
```

### Frontier Detection

Frontier = free cell adjacent to unknown cell

```
1. For each cell in grid:
   - If cell is FREE:
     - Check 4-connected neighbors
     - If any neighbor is UNKNOWN:
       - Add to frontier candidate
2. Cluster connected frontier cells
3. Filter by minimum cluster size
4. Select closest frontier as goal
```

---

## Database Schema

Maps are stored in SQLite with the following schema:

```sql
CREATE TABLE lidar_scans (
    id INTEGER PRIMARY KEY,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    scan_id INTEGER,
    point_count INTEGER,
    points_json TEXT
);

CREATE TABLE slam_map (
    id INTEGER PRIMARY KEY,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    map_name TEXT,
    points_json TEXT
);
```

---

## Hardware Requirements

**For real LiDAR:**
- YDLIDAR X2 / X4 or compatible
- USB Serial connection
- 115200 baud rate

**Simulation mode:**
- No hardware required
- Uses room simulation with walls and obstacles

---

## References

1. **RELAX Paper**: [arXiv:2309.08095](https://arxiv.org/html/2309.08095v2)
   - Reinforcement Learning with 2D LiDAR
   - Frontier-based exploration
   - RRT path planning

2. **Hector SLAM**: Map construction from LiDAR
   - Scan matching
   - Multi-resolution maps

3. **Occupancy Grid Mapping**: Probabilistic Robotics (Thrun et al.)
   - Log-odds representation
   - Bayes update rules

---

## Troubleshooting

**Port already in use:**
```bash
make stop
# Or manually:
lsof -i :8766 -i :808 -i :8767 -i :8081
kill -9 <PID>
```

**Import errors:**
```bash
make install
# or
poetry install
```

**Poetry not installed:**
```bash
curl -sSL https://install.python-poetry.org | python3 -
```

**No data in simulation:**
- Check WebSocket connection in browser console
- Ensure both servers are running
- Try refreshing the page

---

## License

MIT License - See LICENSE file for details.

## Authors

Robot Research Team - PEPEBOT Project