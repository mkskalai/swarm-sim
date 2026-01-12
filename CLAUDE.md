# Drone Swarm Simulation Platform

## Project Overview

A simulation platform for drone swarm research using ArduPilot SITL, Gazebo Harmonic, and ROS2 Jazzy. Designed for researchers and engineers to explore navigation algorithms, machine learning, computer vision, and inter-drone communication.

**Primary Research Focus:**
- Surveillance and reconnaissance
- Target acquisition and engagement
- GPS-denied navigation (visual/inertial odometry)
- Pursuit and tracking algorithms

**Target Configuration:**
- 6-15 quadcopters per simulation
- Fixed RGB camera per drone
- Hybrid control architecture

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     GROUND CONTROL STATION                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  Mission    │  │  Swarm      │  │  Visualization          │  │
│  │  Planner    │  │  Coordinator│  │  (RViz2 / Custom)       │  │
│  └──────┬──────┘  └──────┬──────┘  └─────────────────────────┘  │
│         │                │                                       │
│         └────────┬───────┘                                       │
│                  │ High-level commands                           │
└──────────────────┼───────────────────────────────────────────────┘
                   │
                   │ MAVSDK-Python (async)
                   │
┌──────────────────┼───────────────────────────────────────────────┐
│                  ▼         DRONE SWARM                           │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       ┌─────────┐        │
│  │ Drone 1 │◄─►│ Drone 2 │◄─►│ Drone 3 │ ... │ Drone N │        │
│  └────┬────┘  └────┬────┘  └────┬────┘       └────┬────┘        │
│       │            │            │                  │             │
│       │     P2P Communication (ROS2 Topics)        │             │
│       │     Local coordination, obstacle sharing   │             │
└───────┼────────────┼────────────┼──────────────────┼─────────────┘
        │            │            │                  │
        ▼            ▼            ▼                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SIMULATION LAYER                              │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                    Gazebo Harmonic                         │ │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────────┐  │ │
│  │  │ Physics  │  │ Sensors  │  │ Camera   │  │ World     │  │ │
│  │  │ Engine   │  │ (IMU,GPS)│  │ Plugin   │  │ Models    │  │ │
│  │  └──────────┘  └──────────┘  └──────────┘  └───────────┘  │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │              ArduPilot SITL (per drone)                    │ │
│  │  ardupilot_gazebo plugin ◄─► MAVLink ◄─► MAVSDK           │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### Architecture Decisions Explained

#### 1. MAVSDK-Python vs MAVROS2 vs pymavlink

**We chose: MAVSDK-Python**

| Option | Pros | Cons |
|--------|------|------|
| **MAVSDK-Python** | Clean async API, good swarm support, abstracts MAVLink complexity | Less ROS-native |
| MAVROS2 | Tight ROS2 integration, uses ROS topics/services | Heavy, complex setup, one node per drone |
| pymavlink | Maximum control, lightweight | Verbose, manual message handling |

**Why MAVSDK:** For swarms, we need to manage many connections efficiently. MAVSDK's async design handles this well. We can still publish to ROS2 topics for inter-drone communication while using MAVSDK for flight control.

#### 2. Hybrid Control Architecture

**Centralized (GCS) handles:**
- Mission planning and assignment
- Global coordination (who goes where)
- Monitoring and telemetry aggregation
- High-level decisions (abort, RTL, replan)

**Decentralized (P2P) handles:**
- Local obstacle avoidance
- Formation maintenance
- Neighbor-to-neighbor coordination
- Low-latency reactive behaviors

**Why Hybrid:** Pure centralized fails if GCS connection drops. Pure decentralized is complex for coordinated missions. Hybrid gives us the best of both - structured missions with resilient local behaviors.

#### 3. Gazebo Harmonic + ardupilot_gazebo

**Why this combination:**
- Harmonic is the LTS release matching Jazzy
- ardupilot_gazebo is the official, maintained plugin
- Supports multiple SITL instances (critical for swarms)
- Modern gz-sim transport (not legacy gazebo-classic)

---

## Current Environment

**Host System:**
- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Gazebo Harmonic (gz-sim)
- Python venv: `~venv-ardupilot`

**Installed Components:**
- [x] Gazebo Harmonic
- [x] ArduPilot SITL
- [x] ardupilot_gazebo plugin
- [x] MAVSDK-Python
- [x] Camera sensor configuration
- [ ] ROS2 workspace for swarm
- [ ] Multi-drone world files

---

## Installation Challenges & Solutions

### Challenge 1: Gazebo Version Confusion

**Problem:** There are two Gazebo ecosystems:
- `gazebo-classic` (versions 9, 11) - DEPRECATED
- `gz-sim` (Garden, Harmonic, etc.) - CURRENT

Many tutorials reference the old `gazebo-classic`. ArduPilot documentation sometimes mixes them.

**Solution:**
```bash
# Verify you have gz-sim (new Gazebo), NOT gazebo-classic
gz sim --version  # Should show Harmonic or similar

# If you accidentally have gazebo-classic:
sudo apt remove gazebo*
# Then install gz-sim via official instructions
```

### Challenge 2: ardupilot_gazebo Plugin Discovery

**Problem:** Gazebo can't find the ArduPilot plugin after building.

**Solution:**
```bash
# Add to ~/.bashrc or activate script:
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/path/to/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/path/to/ardupilot_gazebo/models:$HOME/path/to/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

### Challenge 3: Multiple SITL Instances

**Problem:** Running multiple ArduPilot SITL instances requires unique ports.

**Solution:**
```bash
# Each SITL instance needs unique SYSID and ports
# Instance 0 (default): ports 5760, 5762, 5763
# Instance 1: ports 5770, 5772, 5773 (add 10 per instance)

# Launch pattern:
sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 0 -I 0
sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 1 -I 1
```

### Challenge 4: ROS2 Jazzy + Gazebo Transport Bridge

**Problem:** Getting Gazebo camera/sensor data into ROS2 topics.

**Solution:**
```bash
# Install the bridge
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image

# Run bridge for camera (example)
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

### Challenge 5: Python Environment Conflicts

**Problem:** ROS2, MAVSDK, and other packages have conflicting dependencies.

**Solution:**
```bash
# Use the venv but source ROS2 AFTER activating venv
source ~/venv-ardupilot/bin/activate
source /opt/ros/jazzy/setup.bash

# Install MAVSDK in venv
pip install mavsdk
```

### Challenge 6: Performance with Multiple Drones

**Problem:** Gazebo + multiple SITL instances consume significant resources.

**Solution:**
- Use `--headless` mode for SITL when not debugging
- Reduce Gazebo physics rate if needed: `<real_time_factor>0.5</real_time_factor>`
- Disable rendering when not needed: `gz sim -s` (server only)
- Consider distributing SITL instances across CPU cores

### Challenge 7: Hybrid GPU (Intel/NVIDIA) EGL Errors

**Problem:** On laptops with hybrid Intel/NVIDIA graphics, Gazebo's OGRE2 renderer fails with:
```
libEGL warning: egl: failed to create dri2 screen
glx: failed to create dri3 screen
failed to load driver: nouveau
```

The issue is OGRE2 tries to use the Intel GPU's Mesa EGL by default, which fails when camera sensors are present.

**Solution:**
```bash
# Force NVIDIA GPU for rendering
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1

# Then run Gazebo normally
gz sim worlds/single_drone.sdf
```

These variables are automatically set by `scripts/setup_env.sh` when NVIDIA hardware is detected.

### Challenge 8: VSCode Snap Terminal Library Conflicts

**Problem:** When running Gazebo from VSCode's integrated terminal (VSCode installed via snap), you get:
```
gz sim gui: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init
```

The snap-confined VSCode injects its own glibc libraries which conflict with system libraries.

**Solution:**
Option 1: Use an external terminal (Ctrl+Alt+T) instead of VSCode's integrated terminal.

Option 2: Source the setup script which cleans snap paths:
```bash
source scripts/setup_env.sh
gz sim worlds/single_drone.sdf
```

The setup script automatically detects VSCode snap environment and removes conflicting library paths.

### Challenge 9: SITL Frame Selection (JSON vs gazebo-iris)

**Problem:** ArduPilot SITL shows "link 1 down" even when Gazebo is running with the ardupilot_gazebo plugin. Using `-f gazebo-iris` frame results in protocol magic errors.

**Solution:**
The ardupilot_gazebo plugin uses the JSON SITL interface, not the older FDM protocol. Use `-f JSON` instead of `-f gazebo-iris`:

```bash
# WRONG - uses old FDM protocol
sim_vehicle.py -v ArduCopter -f gazebo-iris --console

# CORRECT - uses JSON interface matching ardupilot_gazebo
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f JSON --console --out=udp:127.0.0.1:14540
```

Also ensure Gazebo is started with `-r` flag to run immediately (not paused):
```bash
gz sim -r worlds/single_drone.sdf
```

### Challenge 10: MAVSDK Connection Port

**Problem:** MAVSDK waits forever at "Waiting to discover system on udp://:14540" even though SITL is running.

**Solution:**
By default, `sim_vehicle.py` only outputs MAVLink to port 14550 (for QGroundControl). MAVSDK expects port 14540.

Add the `--out` flag to enable MAVSDK connection:
```bash
# Start SITL with MAVSDK output port
sim_vehicle.py -v ArduCopter -f JSON --console --out=udp:127.0.0.1:14540
```

Or add it dynamically in MAVProxy console:
```
output add 127.0.0.1:14540
```

### Challenge 11: Paths with Spaces in GZ_SIM_RESOURCE_PATH

**Problem:** Gazebo fails with "Unable to find or download file" when project path contains spaces (e.g., "python projects").

**Solution:**
Gazebo's resource path parsing doesn't handle spaces well. Use a symlink:
```bash
ln -s "/path/with spaces/project" ~/project-symlink
```

Then run Gazebo from a directory without spaces:
```bash
cd ~/ardupilot_gazebo
gz sim -r ~/project-symlink/worlds/single_drone.sdf
```

---

## Development Phases

### Phase 1: Single Drone Foundation ✅
**Goal:** Reliable single drone control via MAVSDK

- [x] Install MAVSDK-Python in venv
- [x] Create basic connection test script
- [x] Implement takeoff/land/goto commands
- [x] Add camera feed subscription
- [x] Create world file with RGB camera drone
- [x] Test offboard control mode

**Deliverable:** `swarm/core/drone.py` - Single drone controller class

### Phase 2: Multi-Drone Spawning
**Goal:** Launch N drones with unique identities

- [ ] Create parameterized drone model (SDF/URDF)
- [ ] Build launcher script for N SITL instances
- [ ] Implement drone fleet manager class
- [ ] Handle unique MAVLink system IDs
- [ ] Test 3-drone simultaneous flight

**Deliverable:** `swarm/core/fleet.py` - Fleet spawning and management

### Phase 3: Basic Swarm Coordination
**Goal:** Coordinated movement patterns

- [ ] Implement formation flying (line, V, grid)
- [ ] Add leader-follower mode
- [ ] Create waypoint mission distribution
- [ ] Handle drone failures gracefully
- [ ] Test with 6 drones

**Deliverable:** `swarm/coordination/formations.py`

### Phase 4: ROS2 Integration
**Goal:** P2P communication layer

- [ ] Create ROS2 workspace
- [ ] Define custom messages (DroneState, SwarmCommand)
- [ ] Implement neighbor discovery
- [ ] Add local broadcast for obstacle/target sharing
- [ ] Bridge Gazebo sensors to ROS2

**Deliverable:** `swarm_ros/` - ROS2 package

### Phase 5: Perception Pipeline
**Goal:** Camera-based awareness

- [ ] RGB camera configuration in Gazebo
- [ ] Image processing node (OpenCV)
- [ ] Basic object detection (YOLO or similar)
- [ ] Target tracking across frames
- [ ] Share detections with swarm

**Deliverable:** `swarm/perception/` - Vision processing

### Phase 6: GPS-Denied Navigation
**Goal:** Navigate without GPS

- [ ] Implement visual odometry (ORB-SLAM3 or similar)
- [ ] Fuse IMU + visual odometry
- [ ] Test indoor/GPS-denied scenarios
- [ ] Add landmark-based localization

**Deliverable:** `swarm/navigation/visual_odom.py`

### Phase 7: Advanced Behaviors
**Goal:** Research-ready capabilities

- [ ] Pursuit algorithms (proportional navigation, etc.)
- [ ] Search patterns (lawnmower, spiral, probabilistic)
- [ ] Target handoff between drones
- [ ] Coordinated surveillance coverage

**Deliverable:** `swarm/behaviors/`

### Phase 8: Docker Packaging
**Goal:** One-command deployment

- [ ] Create Dockerfile with all dependencies
- [ ] Handle GPU passthrough for Gazebo rendering
- [ ] Volume mounts for custom worlds/models
- [ ] Docker Compose for multi-container setup
- [ ] Documentation and examples

**Deliverable:** `docker/` - Complete containerization

---

## Project Structure

```
swarm/
├── CLAUDE.md                 # This file
├── README.md                 # User-facing documentation
├── pyproject.toml            # Python project config
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
├── swarm/                    # Main Python package
│   ├── __init__.py
│   ├── core/
│   │   ├── drone.py          # Single drone controller
│   │   ├── fleet.py          # Multi-drone management
│   │   └── config.py         # Configuration management
│   ├── coordination/
│   │   ├── formations.py     # Formation flying
│   │   ├── missions.py       # Mission planning
│   │   └── consensus.py      # Distributed consensus
│   ├── perception/
│   │   ├── camera.py         # Camera interface
│   │   ├── detection.py      # Object detection
│   │   └── tracking.py       # Target tracking
│   ├── navigation/
│   │   ├── visual_odom.py    # Visual odometry
│   │   └── path_planning.py  # Path planners
│   └── behaviors/
│       ├── pursuit.py        # Pursuit algorithms
│       ├── search.py         # Search patterns
│       └── surveillance.py   # Area coverage
├── swarm_ros/                # ROS2 package (Phase 4)
│   ├── package.xml
│   ├── setup.py
│   └── swarm_ros/
├── worlds/                   # Gazebo world files
│   ├── single_drone.sdf
│   ├── multi_drone.sdf
│   └── urban_environment.sdf
├── models/                   # Drone models
│   └── quad_rgb/
│       ├── model.config
│       └── model.sdf
├── scripts/                  # Launch and utility scripts
│   ├── launch_swarm.py
│   └── setup_env.sh
├── tests/
│   └── ...
└── notebooks/                # Jupyter experiments
    └── ...
```

---

## Key Commands Reference

```bash
# Create symlink to avoid path-with-spaces issues (one-time setup)
ln -s "/home/maks/Desktop/python projects/claude/swarm" ~/swarm

# Activate environment
source ~/venv-ardupilot/bin/activate

# Terminal 1: Launch Gazebo (-r flag starts unpaused)
cd ~/ardupilot_gazebo
gz sim -r ~/swarm/worlds/single_drone.sdf

# Terminal 2: Launch ArduPilot SITL
# IMPORTANT: Use -f JSON (not gazebo-iris) and --out for MAVSDK port
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f JSON --console --out=udp:127.0.0.1:14540

# Terminal 3: Run tests
source ~/venv-ardupilot/bin/activate
cd ~/swarm
python scripts/test_connection.py

# Launch multiple SITL (example for 3 drones)
# Each instance needs unique ports: base + (instance * 10)
for i in {0..2}; do
    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -v ArduCopter -f JSON --instance $i -I $i \
        --out=udp:127.0.0.1:$((14540 + i*10)) &
done

# Connect MAVSDK to SITL
# Instance 0: udp://:14540
# Instance 1: udp://:14550
# Instance N: udp://:$((14540 + N*10))

# ROS2-Gazebo bridge (camera example)
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

---

## Testing Strategy

Each phase includes verification:

1. **Unit Tests:** Core logic without simulation
2. **Integration Tests:** Single drone in Gazebo
3. **Swarm Tests:** Multi-drone scenarios
4. **Stress Tests:** Maximum drone count, long duration

```bash
# Run tests
pytest tests/ -v

# Run specific phase tests
pytest tests/test_phase1.py -v
```

---

## Resources

- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [ardupilot_gazebo Repository](https://github.com/ArduPilot/ardupilot_gazebo)
- [MAVSDK-Python Guide](https://mavsdk.mavlink.io/main/en/python/)
- [Gazebo Harmonic Tutorials](https://gazebosim.org/docs/harmonic)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

---

## Notes for Development

- Always test with 1 drone before scaling to N
- Monitor CPU/RAM usage - Gazebo + SITL is heavy
- Use `--speedup` flag for SITL to run faster than real-time
- Log everything during development; reduce logging in production
- Commit working configurations before experimenting

---

## Decisions Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2025-01-12 | MAVSDK-Python over MAVROS2 | Cleaner async API for swarm management |
| 2025-01-12 | Hybrid architecture | Balance between coordination and resilience |
| 2025-01-12 | RGB camera only (initial) | Simplicity; depth can be added later |
| 2025-01-12 | 6-15 drone target | Meaningful swarm behaviors without excessive resources |
