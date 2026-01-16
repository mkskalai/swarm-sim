# User Guide

This guide explains how to run simulations and use the drone swarm platform.

## Prerequisites

Complete the setup in [INSTALL.md](../INSTALL.md) before proceeding.

---

## Quick Start

```bash
# Activate environment (do this in every terminal)
source ~/setup_swarm_env.sh

# Run a 3-drone formation test
python ~/swarm/scripts/run_phase3_test.py --num-drones 3
```

---

## Running Simulations

### All-in-One Test Runner

The easiest way to run simulations:

```bash
# Default: 3 drones, run all tests
python scripts/run_phase3_test.py

# Specify number of drones
python scripts/run_phase3_test.py --num-drones 6

# Run specific test
python scripts/run_phase3_test.py --test formations
python scripts/run_phase3_test.py --test leader_follower
python scripts/run_phase3_test.py --test missions

# Start simulation without running tests (for manual control)
python scripts/run_phase3_test.py --skip-test
```

### Manual Control (Separate Terminals)

For more control, start components separately:

**Terminal 1: Generate world and start Gazebo**
```bash
source ~/setup_swarm_env.sh
python scripts/generate_world.py --num-drones 3
gz sim -r worlds/multi_drone_3.sdf
```

**Terminals 2-4: Start SITL instances**
```bash
source ~/setup_swarm_env.sh
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --out=udp:127.0.0.1:14540 --console
# Repeat with -I1, -I2 and ports 14541, 14542
```

**Terminal 5: Run Python scripts or ROS2**
```bash
source ~/setup_swarm_env.sh
python scripts/test_phase3.py --num-drones 3
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
- 1 = V_FORMATION
- 2 = TRIANGLE
- 3 = GRID
- 4 = CIRCLE
- 5 = DIAMOND

**Land all drones:**
```bash
ros2 service call /swarm/land_all std_srvs/srv/Trigger
```

---

## Formations

The swarm supports 6 formation patterns:

| Formation | Description | Best For |
|-----------|-------------|----------|
| LINE | Drones in a straight line | Corridor surveillance |
| V_FORMATION | Classic V shape | Efficient forward flight |
| TRIANGLE | Equilateral triangle | 3-drone coverage |
| GRID | Square/rectangular grid | Area coverage |
| CIRCLE | Circular arrangement | Perimeter defense |
| DIAMOND | Diamond pattern | Balanced coverage |

### Formation Parameters

- **spacing**: Distance between drones (meters)
- **altitude**: Flight altitude (meters)
- **duration**: How long to hold formation (seconds)

### Example: Formation Transitions

```python
from swarm.coordination import SwarmController, FormationType

controller = SwarmController(num_drones=3)
controller.connect_all()
controller.arm_all()
controller.takeoff_all(altitude=10.0)

# Fly V-formation for 15 seconds
controller.fly_formation(FormationType.V_FORMATION, duration=15.0)

# Transition to circle
controller.fly_formation(FormationType.CIRCLE, duration=15.0)

controller.land_all()
```

---

## Leader-Follower Mode

In leader-follower mode, one drone acts as leader and others maintain relative positions:

```python
controller.start_leader_follower(leader_id=0, formation_type=FormationType.V_FORMATION)

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
    formation=FormationType.V_FORMATION,
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

### Test Detection Standalone

```bash
# Test YOLO detector without simulation
python scripts/test_phase5.py --detector-only
```

---

## Scaling Up

### Adding More Drones

```bash
# Generate models and world for 6 drones
python scripts/generate_world.py --num-drones 6 --spacing 5.0

# Run 6-drone simulation
python scripts/run_phase3_test.py --num-drones 6
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
- Reduce physics rate in world file: `<real_time_factor>0.5</real_time_factor>`

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
