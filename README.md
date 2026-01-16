# Drone Swarm Simulation Platform

A simulation platform for drone swarm research using ArduPilot SITL, Gazebo Harmonic, and ROS2 Jazzy.

## Features

- **Multi-drone simulation** - 3-15 quadcopters with unique MAVLink ports
- **6 formation patterns** - Line, V, Triangle, Grid, Circle, Diamond
- **Leader-follower mode** - Automatic leader promotion on failure
- **ROS2 integration** - P2P communication, neighbor tracking, collision avoidance
- **Perception pipeline** - YOLOv11 object detection with tracking
- **Waypoint missions** - Coordinated multi-drone mission execution

## Requirements

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Gazebo Harmonic (gz-sim)
- ArduPilot SITL + ardupilot_gazebo plugin
- Python 3.12+

## Quick Start

```bash
# Setup environment
source ~/setup_swarm_env.sh

# Generate drone models (first time only)
python scripts/generate_world.py --num-drones 3

# Run 3-drone simulation with formation test
python scripts/run_phase3_test.py --num-drones 3

# Or with ROS2 bridge (separate terminals):
python scripts/run_phase3_test.py --num-drones 3 --skip-test
ros2 launch swarm_ros simulation.launch.py num_drones:=3
```

## Documentation

| Document | Description |
|----------|-------------|
| **[INSTALL.md](INSTALL.md)** | Full installation guide with troubleshooting |
| **[docs/architecture.md](docs/architecture.md)** | System design and component diagrams |
| **[docs/user-guide.md](docs/user-guide.md)** | How to run simulations and use formations |
| **[docs/api/](docs/api/)** | API reference for all modules |
| **[docs/contributing.md](docs/contributing.md)** | Development workflow and code conventions |
| **[CLAUDE.md](CLAUDE.md)** | AI/developer context (architecture decisions, gotchas) |

## Project Structure

```
swarm/
├── swarm/              # Python package (drone control, formations, perception)
├── swarm_ros/          # ROS2 package (P2P communication, services)
├── models/             # Gazebo drone models (DroneTemplate + generated)
├── worlds/             # Gazebo world files
├── scripts/            # Launch and test scripts
└── docs/               # Documentation
```

## Development Status

| Phase | Status | Description |
|-------|--------|-------------|
| 1. Single Drone | ✅ | MAVSDK-based drone controller |
| 2. Multi-Drone | ✅ | Fleet spawning with pymavlink |
| 3. Coordination | ✅ | Formations, leader-follower, missions |
| 4. ROS2 Integration | ✅ | P2P topics, services, SwarmBridge |
| 5. Perception | ✅ | YOLOv11 detection, tracking |
| 6. GPS-Denied Nav | ⬜ | Visual odometry (planned) |
| 7. Behaviors | ⬜ | Pursuit, search patterns (planned) |
| 8. Docker | ⬜ | Containerization (planned) |

## License

MIT
