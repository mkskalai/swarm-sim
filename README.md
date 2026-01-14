# Drone Swarm Simulation Platform

A simulation platform for drone swarm research using ArduPilot SITL, Gazebo Harmonic, and ROS2 Jazzy.

## Features

- **Multi-drone simulation** - 3-15 quadcopters with unique MAVLink ports
- **6 formation patterns** - Line, V, Triangle, Grid, Circle, Diamond
- **Leader-follower mode** - Automatic leader promotion on failure
- **ROS2 integration** - P2P communication, neighbor tracking, collision avoidance
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
source scripts/setup_env.sh

# Generate drone models (first time only)
python scripts/generate_world.py --num-drones 3

# Run 3-drone simulation with formation test
python scripts/run_phase3_test.py --num-drones 3

# Or with ROS2 bridge (separate terminals):
python scripts/run_phase3_test.py --num-drones 3 --skip-test
ros2 launch swarm_ros simulation.launch.py num_drones:=3
```

## Documentation

- **[INSTALL.md](INSTALL.md)** - Full installation guide with troubleshooting
- **[CLAUDE.md](CLAUDE.md)** - Architecture, implementation details, development reference

## Project Structure

```
swarm/
├── swarm/              # Python package (drone control, formations, missions)
├── swarm_ros/          # ROS2 package (P2P communication, services)
├── models/             # Gazebo drone models (DroneTemplate + generated)
├── worlds/             # Gazebo world files
└── scripts/            # Launch and test scripts
```

## License

MIT
