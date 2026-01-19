# Drone Swarm Simulation Platform

A simulation platform for drone swarm research using ArduPilot SITL, Gazebo Harmonic, and ROS2 Jazzy.

## Features

- **Multi-drone simulation** - 3-15 quadcopters with unique MAVLink ports
- **3 formation patterns** - Line, Grid, Star
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

# Run unit tests (no simulation required)
python scripts/run_tests.py --unit

# Run 3-drone simulation test
python scripts/run_tests.py --sim -n 3

# Run with ROS2 bridge
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
| 1. Single Drone | ✅ | pymavlink-based drone controller |
| 2. Multi-Drone | ✅ | Fleet spawning with pymavlink |
| 3. Coordination | ✅ | Formations, leader-follower, missions |
| 4. ROS2 Integration | ✅ | P2P topics, services, SwarmBridge |
| 5. Perception | ✅ | YOLOv11 detection, tracking |
| 6. GPS-Denied Nav | ✅ | ORB+optical flow VIO, EKF fusion |
| 7. Autonomous Missions | ✅ | Pursuit, search patterns, GPS jamming |
| 8. Docker | ✅ | Multi-stage build, dev/ci/headless profiles |
| 9. Tactical Overwatch | ⬜ | [Planned](docs/phase9-tactical-overwatch.md) - Coverage mesh, threat intercept |

## License

MIT
