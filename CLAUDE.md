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

#### 1. pymavlink vs MAVSDK-Python vs MAVROS2

**We chose: pymavlink** (updated from original MAVSDK choice)

| Option | Pros | Cons |
|--------|------|------|
| **pymavlink** | Port isolation works, lightweight, full control | Manual message handling, synchronous |
| MAVSDK-Python | Clean async API, abstracts complexity | Routes by sysid - fails with same-sysid drones |
| MAVROS2 | Tight ROS2 integration | Heavy, complex setup, one node per drone |

**Why pymavlink:** MAVSDK routes commands by MAVLink system ID. When all SITL instances have sysid=1 (default), MAVSDK sends all commands to the same drone. pymavlink uses raw sockets - each connection is isolated by port. For real hardware with onboard computers (1:1 connection), this matches the architecture perfectly.

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

### Challenge 12: MAVProxy --out Parameter and Headless Mode

**Problem:** When running SITL with `--no-mavproxy` (headless mode), the `--out=udp:HOST:PORT` parameter has no effect and MAVSDK cannot connect.

**Root Cause:**
The `--out` parameter is a **MAVProxy** feature, not a SITL feature. It tells MAVProxy to forward MAVLink data to additional endpoints. When using `--no-mavproxy`, there's no MAVProxy running to do the forwarding.

**SITL MAVLink Architecture:**

```
WITH MAVProxy (--console or default):
  SITL → MAVProxy → --out endpoints (UDP)
                  → MAVProxy console
  MAVSDK connects via UDP to --out port

WITHOUT MAVProxy (--no-mavproxy):
  SITL → listens directly on TCP port 5760 + (instance * 10)
  MAVSDK connects via TCP directly to SITL
```

**Solution:**

For headless/automated operation, connect MAVSDK directly to SITL's native TCP ports:

```python
# Instance 0: tcp://127.0.0.1:5760
# Instance 1: tcp://127.0.0.1:5770
# Instance 2: tcp://127.0.0.1:5780
# Instance N: tcp://127.0.0.1:{5760 + N*10}

# WRONG - doesn't work without MAVProxy:
sim_vehicle.py -f JSON -I 0 --out=udp:127.0.0.1:14540 --no-mavproxy

# CORRECT - headless mode, MAVSDK connects to TCP 5760:
sim_vehicle.py -f JSON -I 0 --no-mavproxy
# Then in code: system_address = "tcpout://127.0.0.1:5760"

# ALTERNATIVE - with MAVProxy forwarding:
sim_vehicle.py -f JSON -I 0 --out=udp:127.0.0.1:14540
# Then in code: system_address = "udpin://0.0.0.0:14540"
```

**Port Summary:**

| Mode | SITL Command | MAVSDK Connection |
|------|--------------|-------------------|
| With MAVProxy | `--out=udp:127.0.0.1:14540` | `udpin://0.0.0.0:14540` |
| Headless | `--no-mavproxy` | `tcpout://127.0.0.1:5760` |

**MAVSDK URI Schemes:**
- `udpin://` - Listen for incoming UDP (requires `0.0.0.0` for interface)
- `udpout://` - Send UDP to target
- `tcpin://` - Listen for incoming TCP connections (act as server)
- `tcpout://` - Connect to TCP server (act as client) - **use this for SITL**

### Challenge 13: sim_vehicle.py --speedup Requires Integer

**Problem:** SITL fails silently when `--speedup` is passed a float value like `1.0`.

**Solution:**
Always pass an integer to `--speedup`:
```bash
# WRONG
sim_vehicle.py -f JSON --speedup 1.0

# CORRECT
sim_vehicle.py -f JSON --speedup 1
```

### Debugging Best Practice: Always Log Subprocess Output

When launching subprocesses (like SITL), always capture output to log files instead of using `subprocess.DEVNULL`. Silent failures are extremely difficult to debug.

```python
log_dir = Path("logs")
log_dir.mkdir(exist_ok=True)
stdout_log = open(log_dir / f"process_{i}_stdout.log", "w")
stderr_log = open(log_dir / f"process_{i}_stderr.log", "w")

proc = subprocess.Popen(
    cmd,
    stdout=stdout_log,
    stderr=stderr_log,
)
```

Check logs at `logs/sitl_*_stdout.log` and `logs/sitl_*_stderr.log` when debugging SITL startup issues.

---

## Phase 2 Multi-Drone: SOLVED

After extensive debugging, multi-drone simulation now works. Key solution documented here.

### Working Setup

**Models:** Use pre-configured copies in `~/ardupilot_gazebo/models/`:
- `Drone1` (fdm_port_in=9002, fdm_port_out=9005)
- `Drone2` (fdm_port_in=9012, fdm_port_out=9015)
- `Drone3` (fdm_port_in=9022, fdm_port_out=9025)

**World:** `~/ardupilot_gazebo/worlds/tutorial_multi_drone.sdf` with ODE physics, ogre renderer.

**SITL Launch:**
```bash
# Terminal 1: Gazebo
gz sim -v4 -r ~/ardupilot_gazebo/worlds/tutorial_multi_drone.sdf

# Terminals 2-4: SITL with MAVProxy (provides UDP forwarding)
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --out=udp:127.0.0.1:14540
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I1 --out=udp:127.0.0.1:14541
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I2 --out=udp:127.0.0.1:14542
```

**Flight Script:** `scripts/fly_swarm_pymavlink.py` - uses pymavlink for reliable multi-drone control.

### Key Insights

#### 1. fdm_port_out is Required

Despite being marked "deprecated" in plugin code, `fdm_port_out` is necessary for multi-drone:
```xml
<fdm_port_in>9002</fdm_port_in>
<fdm_port_out>9005</fdm_port_out>  <!-- SITL listens on 9002 + 3 -->
```

SITL calculates: `fdm_port_out = fdm_port_in + 3` (not +1 as some docs suggest).

#### 2. EKF Convergence Required Before Arming

"Need Position Estimate" error means wait for EKF, not just GPS:
```python
def wait_for_ekf(conn, timeout=60.0):
    while time.time() - start < timeout:
        msg = conn.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
        if msg and (msg.flags & 0x18) != 0:  # bits 3 or 4 = position OK
            return True
    return False
```

#### 3. MAVSDK System ID Routing Issue

**Problem:** MAVSDK routes commands by MAVLink system ID. All SITL instances default to sysid=1, so MAVSDK sends all commands to the same drone regardless of port.

**Solution:** Use pymavlink instead. Each `mavlink_connection()` is a separate UDP socket - commands stay isolated by port.

```python
# pymavlink - works (port isolation)
conn0 = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
conn1 = mavutil.mavlink_connection('udpin:0.0.0.0:14541')
# Commands on conn0 go to drone 0, conn1 to drone 1

# MAVSDK - doesn't work with same sysid
drone0 = System(); drone0.connect("udpin://0.0.0.0:14540")
drone1 = System(); drone1.connect("udpin://0.0.0.0:14541")
# Both route to same drone because sysid=1
```

#### 4. Architecture for Real Hardware

For autonomous swarms with onboard computers, each drone talks to its OWN flight controller:
```
┌─────────────────────────────────────────────────────────┐
│                        DRONE N                           │
│  ┌──────────────┐    Serial/USB    ┌─────────────────┐  │
│  │   Onboard    │◄────────────────►│ Flight Controller│  │
│  │  Computer    │     MAVLink      │   (ArduPilot)    │  │
│  │ (Pi/Jetson)  │                  └─────────────────┘  │
│  └──────┬───────┘                                       │
└─────────┼───────────────────────────────────────────────┘
          │ WiFi mesh (application layer)
          ▼
     Other Drones
```

No system ID routing needed - 1:1 connection per drone. pymavlink works perfectly.

### Credits

Multi-drone configuration based on tutorial by [AbdullahArpaci](https://github.com/AbdullahArpaci/ros2-ardupilot-gazebo-harmonic-drone-simulation-tutorial).

---

## Expanding to N Drones

To run simulations with more than 3 drones, you need to create additional drone models and world files. Here's the step-by-step process:

### Step 1: Generate Drone Models

Each drone needs a unique model in `~/ardupilot_gazebo/models/` with a unique `fdm_port_in`. The naming convention is `Drone1`, `Drone2`, etc. (1-indexed).

**Port scheme:** `fdm_port_in = 9002 + (instance_id * 10)`

| Drone | Instance ID | fdm_port_in |
|-------|-------------|-------------|
| Drone1 | 0 | 9002 |
| Drone2 | 1 | 9012 |
| Drone3 | 2 | 9022 |
| Drone4 | 3 | 9032 |
| Drone5 | 4 | 9042 |
| Drone6 | 5 | 9052 |
| DroneN | N-1 | 9002 + (N-1)*10 |

**To create a new drone model manually:**

```bash
# Copy Drone1 as template
cp -r ~/ardupilot_gazebo/models/Drone1 ~/ardupilot_gazebo/models/Drone4

# Edit model.config - change name
sed -i 's/Drone1/Drone4/g' ~/ardupilot_gazebo/models/Drone4/model.config

# Edit model.sdf - change model name and fdm_port_in
sed -i 's/<model name="Drone1">/<model name="Drone4">/g' ~/ardupilot_gazebo/models/Drone4/model.sdf
sed -i 's/<fdm_port_in>9002<\/fdm_port_in>/<fdm_port_in>9032<\/fdm_port_in>/g' ~/ardupilot_gazebo/models/Drone4/model.sdf
```

**Or use the automated generator:**

```bash
python scripts/generate_world.py --num-drones 6
# This creates Drone4, Drone5, Drone6 models automatically
```

### Step 2: Generate World File

The world file includes all drones with their spawn positions. Use the generator:

```bash
python scripts/generate_world.py --num-drones 6 --spacing 5.0
# Output: worlds/multi_drone_6.sdf
```

Options:
- `--num-drones N`: Number of drones (default: 3)
- `--spacing M`: Distance between drones in meters (default: 5.0)
- `--output PATH`: Custom output path

### Step 3: Start SITL Instances

Each drone needs its own SITL instance with MAVProxy for UDP forwarding:

```bash
# Terminal 1: Start Gazebo
gz sim -v4 -r ~/swarm/worlds/multi_drone_6.sdf

# Terminals 2-7: Start 6 SITL instances
for i in {0..5}; do
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I$i \
        --out=udp:127.0.0.1:$((14540 + i)) --console --no-rebuild &
done
```

**SITL UDP port scheme:** Port `14540 + instance_id`

| Drone | Instance | UDP Port (pymavlink) |
|-------|----------|---------------------|
| Drone1 | -I0 | 14540 |
| Drone2 | -I1 | 14541 |
| Drone3 | -I2 | 14542 |
| Drone4 | -I3 | 14543 |
| Drone5 | -I4 | 14544 |
| Drone6 | -I5 | 14545 |

### Step 4: Connect with pymavlink

```python
from pymavlink import mavutil

# Connect to each drone
connections = []
for i in range(6):
    conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{14540 + i}')
    conn.wait_heartbeat()
    connections.append(conn)
    print(f"Drone {i+1} connected")
```

### All-in-One Test Command

The test runner handles all steps automatically:

```bash
python scripts/run_phase3_test.py --num-drones 6 --test formations
```

This will:
1. Generate missing drone models (Drone4, Drone5, Drone6)
2. Generate the world file if missing
3. Start Gazebo
4. Start 6 SITL instances
5. Run the test
6. Clean up all processes

### Resource Requirements

| Drones | Approx. RAM | CPU Cores |
|--------|-------------|-----------|
| 3 | 8 GB | 4 |
| 6 | 12 GB | 6 |
| 10 | 16 GB | 8 |
| 15 | 24 GB | 12 |

Tips for performance:
- Use `--headless` for SITL when not debugging
- Reduce Gazebo physics rate: `<real_time_factor>0.5</real_time_factor>`
- Run Gazebo in server-only mode: `gz sim -s` (no GUI)

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

### Phase 2: Multi-Drone Spawning ✅
**Goal:** Launch N drones with unique identities

- [x] Create parameterized drone model (SDF/Jinja2 template)
- [x] Build launcher script for N SITL instances
- [x] Implement drone fleet manager class
- [x] Handle unique MAVLink system IDs (port scheme: base + instance*10)
- [x] Test 3-drone simultaneous flight

**Deliverable:** `swarm/core/fleet.py` - Fleet spawning and management

### Phase 3: Basic Swarm Coordination ✅
**Goal:** Coordinated movement patterns

- [x] Implement formation flying (line, V, grid, triangle, circle, diamond)
- [x] Add leader-follower mode
- [x] Create waypoint mission distribution
- [x] Handle drone failures gracefully
- [x] Test with 3+ drones

**Deliverable:** `swarm/coordination/` module

### Phase 4: ROS2 Integration ✅
**Goal:** P2P communication layer

- [x] Create ROS2 workspace (`swarm_ros/` package)
- [x] Define custom messages (DroneState, DroneIntent, FormationRole, SwarmStatus)
- [x] Define services (SetFormation, EmergencyCommand)
- [x] Implement neighbor discovery (NeighborTracker)
- [x] Add local broadcast for P2P sharing (position, intent, formation role)
- [x] Create SwarmBridge for simulation (bridges SwarmController to ROS2)
- [x] Create DroneNode for hardware (per-drone controller)
- [x] Create MissionCoordinator for GCS
- [ ] Bridge Gazebo sensors to ROS2 (deferred to Phase 5)

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
│   │   ├── drone.py          # Single drone controller (MAVSDK-based)
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

## Phase 1 Implementation Reference

This section documents the Phase 1 implementation to ease future development.

### Core Module: `swarm/core/drone.py` (477 lines)

**Drone Class** - Single drone controller wrapping MAVSDK:

```python
class Drone:
    def __init__(self, config: Optional[DroneConfig] = None)

    # Connection
    async def connect(self, timeout: float = None) -> bool
    async def disconnect() -> None

    # Basic actions
    async def arm() -> bool
    async def disarm() -> bool
    async def takeoff(altitude: float = None) -> bool
    async def land() -> bool
    async def return_to_launch() -> bool

    # Position control
    async def goto_location(lat, lon, alt, yaw) -> bool  # GPS coordinates
    async def goto_position_ned(n, e, d, yaw, tolerance) -> bool  # Local NED frame

    # Offboard control
    async def start_offboard() -> bool
    async def stop_offboard() -> bool
    async def set_position_ned(n, e, d, yaw) -> bool
    async def set_velocity_ned(vn, ve, vd, yaw_rate) -> bool

    # Properties (cached from telemetry)
    @property position -> Position  # north, east, down, yaw
    @property global_position -> GlobalPosition  # lat, lon, alt
    @property velocity -> Velocity
    @property battery_level -> float
    @property is_connected -> bool
    @property is_armed -> bool
    @property state -> DroneState
```

**DroneState Enum:**
```
DISCONNECTED -> CONNECTED -> ARMED -> TAKING_OFF -> IN_FLIGHT -> LANDING -> LANDED
```

**Telemetry Architecture:**
- Background async tasks continuously poll position, velocity, battery, flight mode
- Latest values cached in properties for instant access
- 20Hz default update rate (configurable)

### Configuration: `swarm/core/config.py` (60 lines)

**DroneConfig:**
```python
@dataclass
class DroneConfig:
    system_address: str = "udp://:14540"
    instance_id: int = 0
    connection_timeout: float = 30.0
    action_timeout: float = 10.0
    default_altitude: float = 10.0
    default_speed: float = 5.0
    position_update_rate: float = 20.0
    velocity_update_rate: float = 20.0

    @classmethod
    def for_instance(cls, instance_id: int, base_port: int = 14540) -> DroneConfig:
        """Port = base_port + (instance_id * 10)"""
```

**SwarmConfig:**
```python
@dataclass
class SwarmConfig:
    num_drones: int = 6
    base_port: int = 14540
    formation_spacing: float = 5.0
    ros_namespace: str = "swarm"

    def get_drone_configs(self) -> list[DroneConfig]
```

### World File: `worlds/single_drone.sdf`

**Structure:**
- Physics: 1ms timestep, real-time factor 1.0
- Plugins: Physics, Sensors (ogre2), UserCommands, SceneBroadcaster, IMU, NavSat
- GPS origin: Canberra, Australia (-35.363262, 149.165237)
- Ground: 500x500m plane
- Drone: `<include><uri>model://iris_with_ardupilot</uri>`

### Drone Model: `~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf`

**ArduPilotPlugin Config (lines 782-879):**
```xml
<plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
  <fdm_addr>127.0.0.1</fdm_addr>
  <fdm_port_in>9002</fdm_port_in>  <!-- JSON SITL interface -->
  <lock_step>1</lock_step>
  <imuName>iris_with_standoffs::imu_link::imu_sensor</imuName>
  <!-- 4 rotor controls with velocity PID, multiplier ±838 -->
</plugin>
```

### Port Scheme

| Instance | FDM Port (Gazebo↔SITL) | MAVSDK Port |
|----------|------------------------|-------------|
| 0        | 9002                   | 14540       |
| 1        | 9012                   | 14550       |
| 2        | 9022                   | 14560       |
| N        | 9002 + N*10            | 14540 + N*10|

### Camera: `swarm/perception/camera.py` (202 lines)

Interface layer for Gazebo cameras:
- `GazeboCamera.list_available_topics()` - uses `gz topic -l`
- `GazeboCamera.is_publishing()` - checks camera activity
- `get_ros2_bridge_command()` - generates bridge command for ROS2

**Note:** Direct gz-transport Python bindings unavailable; use ROS2 bridge.

### Test Scripts

| Script | Purpose |
|--------|---------|
| `scripts/test_connection.py` | Connection, telemetry, takeoff/land, goto patterns |
| `scripts/test_offboard.py` | Offboard position/velocity control, square/circle patterns |
| `scripts/setup_env.sh` | Environment setup (venv, paths, GPU, snap cleanup) |

### Dependencies (pyproject.toml)

```toml
dependencies = [
    "mavsdk>=2.0.0",
    "numpy>=1.24.0",
    "jinja2>=3.1.0",
]
# Optional: opencv-python for vision
```

---

## Phase 2 Implementation Reference

This section documents the Phase 2 implementation (Multi-Drone Spawning).

### Fleet Manager: `swarm/core/fleet.py`

**Fleet Class** - Manages multiple Drone instances with concurrent operations:

```python
class Fleet:
    def __init__(self, config: Optional[FleetConfig] = None)

    # Concurrent operations
    async def connect_all(timeout: float = 30.0) -> bool
    async def disconnect_all() -> None
    async def arm_all() -> bool
    async def disarm_all() -> bool
    async def takeoff_all(altitude: float = 10.0) -> bool
    async def land_all() -> bool
    async def return_to_launch_all() -> bool

    # Offboard control
    async def start_offboard_all() -> bool
    async def stop_offboard_all() -> bool
    async def goto_positions(positions: list[Position], tolerance: float) -> bool
    async def set_positions(positions: list[Position]) -> bool

    # Status
    def get_status() -> FleetStatus
    def get_positions() -> list[Optional[Position]]
    def get_battery_levels() -> list[float]

    # Container protocol
    def __len__() -> int
    def __getitem__(index: int) -> Drone
    def __iter__() -> Iterator[Drone]
```

**FleetState Enum:**
```
UNINITIALIZED -> CONNECTING -> READY -> ARMED -> IN_FLIGHT -> LANDING -> ERROR
```

**FleetConfig:**
```python
@dataclass
class FleetConfig:
    num_drones: int = 3
    base_mavsdk_port: int = 14540
    base_fdm_port: int = 9002
    port_offset: int = 10
    formation_spacing: float = 5.0

    def get_drone_config(instance_id: int) -> DroneConfig
    def get_fdm_port(instance_id: int) -> int
    def get_mavsdk_port(instance_id: int) -> int
    def get_spawn_position(instance_id: int) -> tuple[float, float, float]
```

### World Generation

**Template:** `worlds/multi_drone.sdf.jinja`
- Jinja2 template for N-drone worlds
- Each drone gets unique `fdm_port_in` in ArduPilotPlugin

**Generator:** `scripts/generate_world.py`
```bash
python scripts/generate_world.py --num-drones 3 --spacing 5.0
# Outputs: worlds/multi_drone_3.sdf
```

### SITL Launcher: `scripts/launch_sitl.py`

**SITLLauncher Class:**
```python
class SITLLauncher:
    def __init__(self, num_instances: int = 3)
    def launch_all(headless: bool = True, speedup: float = 1.0) -> bool
    def shutdown_all() -> None
    def get_status() -> dict
```

**Usage:**
```bash
python scripts/launch_sitl.py --num-instances 3 --headless
```

### Multi-Drone Commands Reference

```bash
# Generate 3-drone world
python scripts/generate_world.py --num-drones 3

# Terminal 1: Start Gazebo
cd ~/ardupilot_gazebo
gz sim -r ~/swarm/worlds/multi_drone_3.sdf

# Terminal 2: Start SITL instances
python scripts/launch_sitl.py --num-instances 3

# Terminal 3: Run fleet test
python scripts/test_fleet.py --num-drones 3
```

### Test Scripts (Phase 2)

| Script | Purpose |
|--------|---------|
| `scripts/generate_world.py` | Generate multi-drone world files |
| `scripts/launch_sitl.py` | Launch/manage multiple SITL instances |
| `scripts/test_fleet.py` | Integration test for fleet operations |
| `tests/test_phase2.py` | Unit tests for FleetConfig, Fleet |

---

## Phase 3 Implementation Reference

This section documents the Phase 3 implementation (Swarm Coordination).

### Coordination Module: `swarm/coordination/`

**SwarmController** - Main orchestrator using pymavlink:
```python
class SwarmController:
    def __init__(self, config: Optional[SwarmConfig] = None)

    # Connection management
    def connect_all(timeout: float = None) -> bool
    def disconnect_all() -> None

    # Basic flight commands
    def wait_for_ekf_all(timeout: float = None) -> bool
    def set_mode_all(mode: str) -> bool
    def arm_all() -> bool
    def disarm_all() -> None
    def takeoff_all(altitude: float = 10.0, wait: bool = True) -> bool
    def land_all() -> None
    def return_to_launch_all() -> None

    # Formation flying (waits for arrival, then holds)
    def fly_formation(
        formation_type: FormationType,
        duration: float = 15.0,
        config: FormationConfig = None,
        arrival_timeout: float = 30.0,
        arrival_tolerance: float = 2.0,
    ) -> bool
    def fly_formation_transition(
        from_type: FormationType,
        to_type: FormationType,
        transition_duration: float = 5.0,
        hold_duration: float = 10.0,
    ) -> bool

    # Leader-follower mode
    def start_leader_follower(leader_id: int, formation_type: FormationType) -> None
    def update_leader_follower(
        leader_position: PositionNED,
        duration: float = None,
        wait_for_arrival: bool = True,
    ) -> None

    # Mission execution
    def execute_mission(mission: SwarmMission) -> bool
```

**FormationType Enum:**
```
LINE, V_FORMATION, TRIANGLE, GRID, CIRCLE, DIAMOND
```

**SwarmConfig:**
```python
@dataclass
class SwarmConfig:
    num_drones: int = 3
    base_port: int = 14540
    heartbeat_timeout: float = 3.0
    position_update_rate: float = 4.0  # Hz
    connection_timeout: float = 30.0
    ekf_timeout: float = 60.0
```

### Key Design: Position Arrival Verification

All formation and movement commands now:
1. Send position commands to drones
2. **Wait for drones to reach target positions** (within tolerance)
3. Then hold/continue for the specified duration

This fixes the issue where drones were never verified to have actually moved.

### All-in-One Test Runner: `scripts/run_phase3_test.py`

Manages complete test lifecycle: Gazebo → SITL instances → Tests → Cleanup

**Prerequisites:**
```bash
# Build ArduPilot (one-time, or after code changes)
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

**Usage:**
```bash
# Default: 3 drones, all tests
python scripts/run_phase3_test.py

# 6 drones
python scripts/run_phase3_test.py --num-drones 6

# Run only formations test
python scripts/run_phase3_test.py --num-drones 3 --test formations

# Skip Gazebo if already running manually
python scripts/run_phase3_test.py --skip-gazebo

# Skip both Gazebo and SITL if already running
python scripts/run_phase3_test.py --skip-gazebo --skip-sitl

# Custom startup delay between SITL launches
python scripts/run_phase3_test.py --num-drones 6 --startup-delay 8
```

### Manual Testing (Separate Terminals)

**Terminal 1: Start Gazebo**
```bash
gz sim -v4 -r ~/ardupilot_gazebo/worlds/tutorial_multi_drone.sdf
```

**Terminals 2-4: Start SITL instances**
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --out=udp:127.0.0.1:14540 --console --no-rebuild
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I1 --out=udp:127.0.0.1:14541 --console --no-rebuild
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I2 --out=udp:127.0.0.1:14542 --console --no-rebuild
```

**Terminal 5: Run test**
```bash
python scripts/test_phase3.py --num-drones 3
```

### Test Scripts (Phase 3)

| Script | Purpose |
|--------|---------|
| `scripts/run_phase3_test.py` | All-in-one test runner (Gazebo + SITL + tests) |
| `scripts/test_phase3.py` | Integration test (requires running Gazebo/SITL) |
| `scripts/fly_swarm_pymavlink.py` | Standalone formation flight demo |
| `tests/test_phase3.py` | Unit tests (no simulation required) |

### Coordination Submodules

| Module | Purpose |
|--------|---------|
| `formations.py` | Formation calculations (LINE, V, TRIANGLE, GRID, CIRCLE, DIAMOND) |
| `leader_follower.py` | Leader-follower mode with auto-promotion on failure |
| `missions.py` | Waypoint mission planning and execution |
| `failure_handler.py` | Drone failure detection and graceful degradation |
| `swarm_controller.py` | Main pymavlink-based orchestrator |

---

## Phase 4 Implementation Reference

This section documents the Phase 4 implementation (ROS2 Integration).

### Architecture: Hybrid Approach

**Simulation mode:** SwarmBridge wraps existing SwarmController and exposes it via ROS2 topics/services. No changes to proven pymavlink coordination.

**Hardware mode:** DroneNode runs on each drone's onboard computer with local pymavlink to flight controller. P2P communication via ROS2 topics.

```
Simulation:                          Hardware:
┌─────────────┐                     ┌─────────────┐
│ SwarmBridge │──ROS2 topics───►    │   GCS       │
│  (wraps     │                     │ Mission     │
│ SwarmCtrl)  │                     │ Coordinator │
└──────┬──────┘                     └──────┬──────┘
       │pymavlink                          │ROS2
       ▼                                   ▼
  SITL instances                    ┌──────────────┐
                                    │  DroneNode   │◄──P2P──►NeighborNodes
                                    │ +pymavlink   │
                                    └──────┬───────┘
                                           │serial
                                           ▼
                                    Flight Controller
```

### Package Structure: `swarm_ros/`

```
swarm_ros/
├── package.xml                  # ament_cmake package manifest
├── CMakeLists.txt               # Build configuration (message gen + Python)
├── msg/
│   ├── DroneState.msg           # Position, velocity, battery, health
│   ├── DroneIntent.msg          # Target position, action type
│   ├── FormationRole.msg        # Leader/follower role, offset
│   └── SwarmStatus.msg          # Aggregated swarm status
├── srv/
│   ├── SetFormation.srv         # Request formation change
│   └── EmergencyCommand.srv     # Emergency stop/RTL/land
├── swarm_ros/
│   ├── __init__.py
│   ├── neighbor_tracker.py      # P2P neighbor discovery & collision avoidance
│   ├── swarm_bridge.py          # Bridges SwarmController to ROS2 (simulation)
│   ├── drone_node.py            # Per-drone controller (hardware)
│   └── mission_coordinator.py   # GCS coordinator
├── launch/
│   ├── simulation.launch.py     # Launch SwarmBridge
│   └── drone.launch.py          # Launch DroneNode
├── config/
│   └── swarm_params.yaml        # ROS2 parameters
├── scripts/                     # Entry point executables
└── test/
    └── test_neighbor_tracker.py # Unit tests
```

### Custom Messages

**DroneState.msg** (published @ 10Hz per drone):
```
uint8 drone_id
float32[3] position_ned     # North, East, Down (meters)
float32[3] velocity_ned     # m/s
float32 yaw_deg
uint8 state                 # DISCONNECTED=0, CONNECTED=1, ARMED=2, IN_FLIGHT=4, etc.
uint8 battery_percent
bool is_healthy, has_gps_lock, ekf_ok
builtin_interfaces/Time stamp
```

**DroneIntent.msg** (published @ 4Hz):
```
uint8 drone_id
float32[3] target_position_ned
uint8 action_type           # HOVER=0, GOTO=1, ORBIT=2, TRACK=3, RTL=5, LAND=6
uint16 action_id
float32 eta_seconds
```

**FormationRole.msg** (published @ 4Hz):
```
uint8 drone_id
uint16 formation_id
uint8 role                  # UNASSIGNED=0, LEADER=1, FOLLOWER=2
uint8 leader_id
float32[3] offset_from_leader
uint8 formation_type        # LINE=0, V=1, TRIANGLE=2, GRID=3, CIRCLE=4, DIAMOND=5
```

### Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/swarm/status` | SwarmStatus | 1Hz | Aggregated swarm status (from bridge or coordinator) |
| `/drone_N/state` | DroneState | 10Hz | Per-drone position/health (P2P sharing) |
| `/drone_N/intent` | DroneIntent | 4Hz | Per-drone intentions (P2P sharing) |
| `/drone_N/formation_role` | FormationRole | 4Hz | Formation membership (P2P sharing) |
| `/drone_N/formation_command` | FormationRole | on demand | Role assignment from coordinator |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/swarm/connect` | Trigger | Connect to drones (simulation only) |
| `/swarm/set_formation` | SetFormation | Assign formation to swarm |
| `/swarm/takeoff_all` | Trigger | Takeoff all drones |
| `/swarm/land_all` | Trigger | Land all drones |
| `/swarm/arm_all` | Trigger | Arm all drones |
| `/swarm/emergency` | EmergencyCommand | Emergency commands (RTL, land, hover) |
| `/drone_N/emergency` | EmergencyCommand | Per-drone emergency (hardware) |

### Key Commands

```bash
# Build the package (from ROS2 workspace)
cd ~/ros2_ws  # or wherever your workspace is
ln -s /path/to/swarm/swarm_ros src/swarm_ros
colcon build --packages-select swarm_ros
source install/setup.bash

# Simulation mode (requires running Gazebo + SITL)
# Terminal 1: Start simulation
python scripts/run_phase3_test.py --num-drones 3 --skip-test

# Terminal 2: Start ROS2 bridge
ros2 launch swarm_ros simulation.launch.py num_drones:=3

# Terminal 3: Monitor
ros2 topic echo /swarm/status
ros2 topic echo /drone_0/state

# Terminal 4: Command formation
ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation \
  "{formation_type: 1, spacing: 5.0, altitude: 10.0, duration: 30.0}"

# Hardware mode (on each drone's onboard computer)
ros2 launch swarm_ros drone.launch.py \
  drone_id:=0 num_drones:=6 serial_port:=/dev/ttyACM0

# Hardware mode with SITL (for testing)
ros2 launch swarm_ros drone.launch.py \
  drone_id:=0 num_drones:=3 use_udp:=true udp_port:=14540
```

### NeighborTracker

Core module for P2P awareness and collision avoidance:

```python
from swarm_ros import NeighborTracker, NeighborTrackerConfig

# On DroneNode initialization
config = NeighborTrackerConfig(
    stale_timeout=3.0,      # Mark neighbor stale after 3s
    failed_timeout=10.0,    # Mark neighbor failed after 10s
    collision_horizon=5.0,  # Look ahead 5s for collisions
    safe_distance=3.0,      # Minimum safe distance (meters)
)
tracker = NeighborTracker(node, own_drone_id=0, num_drones=6, config=config)

# Register callbacks
tracker.on_neighbor_joined(lambda did: print(f"Neighbor {did} joined"))
tracker.on_neighbor_lost(lambda did: print(f"Neighbor {did} lost"))
tracker.on_collision_warning(lambda w: print(f"Collision with {w.other_drone_id}!"))

tracker.start()  # Start subscriptions

# In control loop
collision = tracker.predict_collision(own_position, own_velocity)
if collision and collision.time_to_collision < 2.0:
    # Execute avoidance maneuver using collision.avoidance_vector
    pass
```

### Code Reuse

| Existing Module | Reuse in swarm_ros |
|-----------------|-------------------|
| `formations.py` | 100% - Direct import for position calculations |
| `missions.py` | 80% - Import Waypoint, MissionPlanner |
| `leader_follower.py` | 60% - Logic distributed across DroneNodes |
| `failure_handler.py` | 70% - NeighborTracker handles peer failures |
| `swarm_controller.py` | Via SwarmBridge for simulation |

### Phase 4 Bug Fixes

This section documents critical fixes made during Phase 4 development.

#### Fix 1: Python Module Installation Path (CMakeLists.txt)

**Problem:** After `colcon build`, ROS2 couldn't find `swarm_ros` Python modules:
```
ModuleNotFoundError: No module named 'swarm_ros.swarm_bridge'
```

**Root Cause:** The original CMakeLists.txt installed Python modules to `lib/python3/dist-packages/`, but ROS2's rosidl generates message bindings to `lib/python3.12/site-packages/`. Python only searches one path.

**Fix:** Use versioned Python path in CMakeLists.txt:

```cmake
# Before (WRONG)
install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
)

# After (CORRECT)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
```

The `Python3_VERSION_MAJOR` and `Python3_VERSION_MINOR` variables come from `find_package(Python3)` and ensure the path matches the system Python version (3.12 on Ubuntu 24.04).

#### Fix 2: PYTHONPATH for Main Swarm Package (Launch File)

**Problem:** SwarmBridge couldn't import the main `swarm` package:
```
ModuleNotFoundError: No module named 'swarm'
```

**Root Cause:** The `swarm` Python package is not installed via pip or ROS2 - it's a local development package. ROS2 launch doesn't inherit the full environment.

**Fix:** Set PYTHONPATH in the launch file:

```python
# In simulation.launch.py
import os
from pathlib import Path

SWARM_PROJECT_ROOT = str(Path.home() / "Desktop" / "python projects" / "claude" / "swarm")

def generate_launch_description():
    # ... other setup ...

    current_pythonpath = os.environ.get("PYTHONPATH", "")
    new_pythonpath = f"{SWARM_PROJECT_ROOT}:{current_pythonpath}" if current_pythonpath else SWARM_PROJECT_ROOT

    set_pythonpath = SetEnvironmentVariable(
        name="PYTHONPATH",
        value=new_pythonpath,
    )

    return LaunchDescription([
        # ... arguments ...
        set_pythonpath,  # Must come before node
        swarm_bridge_node,
    ])
```

#### Fix 3: Formation Type Integer-to-Enum Mapping

**Problem:** ROS2 service call failed with:
```
Invalid formation type: 1. Valid: line, v, triangle...
```

**Root Cause:** `SetFormation.srv` uses integer `formation_type` (0-5), but the Python `FormationType` enum uses string values ("line", "v_formation", etc.). Direct lookup failed.

**Fix:** Add mapping dictionary in SwarmBridge:

```python
def _handle_set_formation(self, request, response):
    # Map integer formation type to FormationType enum
    formation_map = {
        0: self._FormationType.LINE,
        1: self._FormationType.V_FORMATION,
        2: self._FormationType.TRIANGLE,
        3: self._FormationType.GRID,
        4: self._FormationType.CIRCLE,
        5: self._FormationType.DIAMOND,
    }

    if request.formation_type not in formation_map:
        response.success = False
        response.message = f"Invalid formation type: {request.formation_type}. Valid: 0=LINE, 1=V, 2=TRIANGLE, 3=GRID, 4=CIRCLE, 5=DIAMOND"
        return response

    formation_type = formation_map[request.formation_type]
    # ... rest of handler
```

#### Fix 4: Populate drone_states Array in SwarmStatus

**Problem:** `/swarm/status` topic showed empty `drone_states: []` array.

**Root Cause:** `_publish_swarm_status()` was not populating the `drone_states` array field.

**Fix:** Build drone state messages in the publish function:

```python
def _publish_swarm_status(self) -> None:
    msg = self._SwarmStatus()
    # ... populate counters ...

    # Build drone_states array
    now = self.get_clock().now().to_msg()
    for drone_id in range(self.num_drones):
        drone_msg = self._DroneState()
        drone_msg.drone_id = drone_id

        if drone_id in positions:
            pos = positions[drone_id]
            drone_msg.position_ned = [pos[0], pos[1], pos[2]]
        else:
            drone_msg.position_ned = [0.0, 0.0, 0.0]

        # ... populate other fields ...
        drone_msg.stamp = now
        msg.drone_states.append(drone_msg)  # Add to array

    self.status_pub.publish(msg)
```

---

## Decisions Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2025-01-12 | MAVSDK-Python over MAVROS2 | Cleaner async API for swarm management |
| 2025-01-12 | Hybrid architecture | Balance between coordination and resilience |
| 2025-01-12 | RGB camera only (initial) | Simplicity; depth can be added later |
| 2025-01-12 | 6-15 drone target | Meaningful swarm behaviors without excessive resources |
| 2025-01-14 | pymavlink over MAVSDK | MAVSDK routes by sysid causing multi-drone issues; pymavlink uses port isolation |
| 2025-01-14 | Add fdm_port_out to models | Required for multi-drone despite being "deprecated" in plugin code |
| 2026-01-14 | Hybrid ROS2 architecture | SwarmBridge for simulation (preserves working code), DroneNode for hardware (true P2P) |
| 2026-01-14 | P2P data: position+intent+formation | Sufficient for coordination without bandwidth overhead of full perception sharing |
