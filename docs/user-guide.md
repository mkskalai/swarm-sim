# User Guide

This guide explains how to run simulations and use the drone swarm platform.

## Prerequisites

Complete the setup in [INSTALL.md](../INSTALL.md) before proceeding.

---

## Quick Start

### From Source

```bash
# Activate environment (do this in every terminal)
source ~/setup_swarm_env.sh

# Run unit tests (fast, no simulation)
python scripts/run_tests.py --unit

# Run a 3-drone simulation test
python scripts/run_tests.py --sim -n 3
```

### In Docker

```bash
cd docker

# Build the image first (one-time)
./scripts/build.sh

# Run unit tests
./scripts/run.sh ci

# Run 3-drone simulation
./scripts/run.sh sim 3

# Interactive development shell
./scripts/run.sh dev
```

---

## Running Tests

### Unified Test Runner

The unified test runner handles simulation lifecycle automatically:

```bash
# Unit tests only (fast, no simulation)
python scripts/run_tests.py --unit

# Simulation tests with N drones
python scripts/run_tests.py --sim -n 3
python scripts/run_tests.py --sim -n 6

# All tests (unit + simulation)
python scripts/run_tests.py --all -n 3

# Different world types
python scripts/run_tests.py --sim -n 3 --world basic    # Minimal (ground + sun)
python scripts/run_tests.py --sim -n 3 --world complex  # Full (people, trucks, buildings)

# Different spawn layouts
python scripts/run_tests.py --sim -n 6 --layout grid    # sqrt(N) x sqrt(N) grid
python scripts/run_tests.py --sim -n 6 --layout line    # Drones in a row

# Skip simulation startup (if already running)
python scripts/run_tests.py --sim -n 3 --skip-sim

# Run without ROS2 (algorithm testing only)
python scripts/run_tests.py --sim -n 3 --no-ros

# Increase timeouts for slow hardware (CPU-only or slow GPU)
python scripts/run_tests.py --sim -n 3 --timeout-multiplier 3.0

# Simulation speed (2x faster than real-time)
python scripts/run_tests.py --sim -n 3 --speed 2.0
```

### Docker Commands

```bash
cd docker

# Unit tests (CI mode)
./scripts/run.sh ci

# Simulation tests
./scripts/run.sh sim 3        # 3 drones with GUI
./scripts/run.sh sim-gpu 3    # 3 drones with GPU acceleration
./scripts/run.sh sim-headless 3  # Headless (no GUI)

# All tests
./scripts/run.sh all 3

# Interactive shell
./scripts/run.sh dev          # Development mode with source mounts
./scripts/run.sh gpu          # GPU-enabled development shell

# Run specific pytest path
./scripts/run.sh test tests/unit/
```

---

## Manual Control (Separate Terminals)

For more control, start components separately:

**Terminal 1: Generate world and start Gazebo**
```bash
source ~/setup_swarm_env.sh
python scripts/generate_world.py -n 3
gz sim -r worlds/generated_3.sdf
```

**Terminals 2-4: Start SITL instances**
```bash
source ~/setup_swarm_env.sh
python scripts/launch_sitl.py -n 3
```

Or start each SITL manually:
```bash
sim_vehicle.py -v ArduCopter -f JSON --model JSON -I0 --out=udp:127.0.0.1:14540 --console
# Repeat with -I1, -I2 and ports 14541, 14542
```

**Terminal 5: Run Python scripts or ROS2**
```bash
source ~/setup_swarm_env.sh
ros2 launch swarm_ros simulation.launch.py num_drones:=3
```

---

## ROS2 Interface

### Starting the ROS2 Bridge

After starting the simulation:

```bash
source ~/setup_swarm_env.sh
ros2 launch swarm_ros simulation.launch.py num_drones:=3
```

### Monitoring Topics

```bash
# List all topics
ros2 topic list

# Monitor swarm status
ros2 topic echo /swarm/status

# Monitor specific drone
ros2 topic echo /drone_0/state
```

### Controlling the Swarm via Services

**Connect to drones:**
```bash
ros2 service call /swarm/connect std_srvs/srv/Trigger
```

**Arm all drones:**
```bash
ros2 service call /swarm/arm_all std_srvs/srv/Trigger
```

**Takeoff:**
```bash
ros2 service call /swarm/takeoff_all std_srvs/srv/Trigger
```

**Fly a formation:**
```bash
ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation \
  "{formation_type: 1, spacing: 5.0, altitude: 10.0, duration: 30.0}"
```

Formation types:
- 0 = LINE
- 1 = GRID
- 2 = STAR

**Land all drones:**
```bash
ros2 service call /swarm/land_all std_srvs/srv/Trigger
```

---

## Formations

The swarm supports 3 formation patterns:

| Formation | Description | Best For |
|-----------|-------------|----------|
| LINE | Drones in a straight line along East axis | Corridor surveillance |
| GRID | Square/rectangular grid (default) | Area coverage |
| STAR | Star pattern in YZ plane (East-Down) | Visual displays, altitude-separated coverage |

### Formation Parameters

- **spacing**: Distance between drones (meters)
- **altitude**: Flight altitude (meters)
- **duration**: How long to hold formation (seconds)

### Example: Formation Transitions

```python
from swarm.coordination import SwarmController, SwarmConfig, FormationType, FormationConfig

config = SwarmConfig(num_drones=3)
controller = SwarmController(config)
controller.connect_all()
controller.arm_all()
controller.takeoff_all(altitude=10.0)

# Fly line formation for 15 seconds
formation_config = FormationConfig(spacing=5.0, altitude=10.0)
controller.fly_formation(FormationType.LINE, duration=15.0, config=formation_config)

# Transition to star
controller.fly_formation(FormationType.STAR, duration=15.0, config=formation_config)

controller.land_all()
```

---

## Leader-Follower Mode

In leader-follower mode, one drone acts as leader and others maintain relative positions:

```python
controller.start_leader_follower(leader_id=0, formation_type=FormationType.GRID)

# Move the formation by moving the leader
controller.update_leader_follower(
    leader_position=PositionNED(north=50.0, east=0.0, down=-10.0),
    wait_for_arrival=True
)
```

If the leader fails, the next drone automatically promotes to leader.

---

## Waypoint Missions

Define multi-waypoint missions for coordinated flight:

```python
from swarm.coordination import SwarmMission, Waypoint

mission = SwarmMission(
    waypoints=[
        Waypoint(north=0, east=0, down=-10, hold_time=5.0),
        Waypoint(north=50, east=0, down=-10, hold_time=5.0),
        Waypoint(north=50, east=50, down=-10, hold_time=5.0),
        Waypoint(north=0, east=50, down=-10, hold_time=5.0),
    ],
    formation=FormationType.GRID,
    formation_spacing=5.0,
)

controller.execute_mission(mission)
```

---

## Perception Pipeline

### Enable Cameras

Start simulation with camera bridging:

```bash
ros2 launch swarm_ros simulation.launch.py num_drones:=3 enable_cameras:=true
```

### Start Perception Nodes

```bash
ros2 launch swarm_ros perception.launch.py num_drones:=3
```

### Monitor Detections

```bash
ros2 topic echo /drone_0/detections
```

---

## Scaling Up

### Adding More Drones

```bash
# Generate models and world for 6 drones
python scripts/generate_world.py -n 6 --spacing 5.0

# Run 6-drone simulation
python scripts/run_tests.py --sim -n 6
```

### Resource Requirements

| Drones | RAM | CPU Cores |
|--------|-----|-----------|
| 3 | 8 GB | 4 |
| 6 | 12 GB | 6 |
| 10 | 16 GB | 8 |
| 15 | 24 GB | 12 |

### Performance Tips

- Use `--headless` for SITL when not debugging
- Run Gazebo server-only: `gz sim -s` (no GUI)
- Simulation speed: `--speed 2.0` runs 2x faster than real-time (default: 1.0)
- For slow hardware, use `--speed 0.5` to reduce load

---

## Troubleshooting

See [INSTALL.md - Common Issues](../INSTALL.md#common-issues) for setup problems.

### Simulation Issues

**Drones don't move:**
- Check SITL is running and linked: Look for "Link 0 OK" in SITL console
- Ensure Gazebo started with `-r` flag (not paused)

**"Need Position Estimate" on arming:**
- EKF hasn't converged yet
- Wait 30-60 seconds after SITL startup

**ROS2 services timeout:**
- Ensure SwarmBridge is running
- Check `ros2 topic list` shows expected topics

**Cameras not publishing:**
- Check `gz topic -l | grep camera` shows camera topics
- Verify ros_gz_bridge is running
