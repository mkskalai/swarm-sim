# Installation Guide

Complete setup guide for the Drone Swarm Simulation Platform. Follow these steps in order on a fresh Ubuntu 24.04 system.

## System Requirements

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 24.04 LTS (Noble Numbat) |
| **ROS2** | Jazzy Jalisco |
| **Gazebo** | Harmonic (gz-sim 8.x) |
| **ArduPilot** | Copter-4.5 |
| **Python** | 3.12 |

**Hardware:**
- 16GB+ RAM recommended for multi-drone simulation
- NVIDIA GPU recommended for Gazebo rendering (Intel works but slower)
- 50GB+ free disk space

---

## Step 1: System Prerequisites

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3-pip \
    python3-venv \
    lsb-release \
    gnupg
```

---

## Step 2: Install Gazebo Harmonic

Reference: https://gazebosim.org/docs/harmonic/install_ubuntu/

```bash
# Add Gazebo package repository
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update

# Install Gazebo Harmonic
sudo apt install -y gz-harmonic

# Verify installation
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

---

## Step 3: Install ROS2 Jazzy

Reference: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Install ROS2 Jazzy Desktop (full)
sudo apt install -y ros-jazzy-desktop

# Install ROS2-Gazebo bridge packages
sudo apt install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim

# Install additional ROS2 development tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

---

## Step 4: Clone and Build ArduPilot

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Checkout stable Copter release
git checkout Copter-4.5
git submodule update --init --recursive

# Install ArduPilot dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload profile (or log out and back in)
. ~/.profile

# Build ArduPilot SITL
./waf configure --board sitl
./waf copter

# Verify build
cd ArduCopter
sim_vehicle.py --help
# Should display sim_vehicle.py help text
```

---

## Step 5: Install ArduPilot Gazebo Plugin

```bash
# Install plugin dependencies
sudo apt install -y \
    libgz-sim8-dev \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl

# Clone ardupilot_gazebo
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo

# Build the plugin
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

---

## Step 6: Create Python Virtual Environment

```bash
# Create venv in home directory
cd ~
python3 -m venv venv-ardupilot

# Activate venv
source ~/venv-ardupilot/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install swarm project dependencies (after cloning in Step 8)
# pip install -r ~/swarm/requirements.txt
```

---

## Step 7: Setup Environment Variables

The project includes a setup script at `scripts/setup_env.sh`. Create a symlink for easy access:

```bash
# After cloning (see Step 8), create symlink:
ln -s ~/swarm/scripts/setup_env.sh ~/setup_swarm_env.sh
```

Add to your `~/.bashrc` for automatic sourcing (optional):

```bash
echo 'source ~/setup_swarm_env.sh' >> ~/.bashrc
```

The setup script configures:
- Python virtual environment (`~/venv-ardupilot`)
- ROS2 Jazzy
- Gazebo plugin and resource paths
- NVIDIA GPU support (for hybrid graphics)
- VSCode snap library path cleanup
- PYTHONPATH for the swarm module

---

## Step 8: Clone the Swarm Project

```bash
cd ~
git clone https://github.com/mkskalai/swarm-sim.git swarm
cd ~/swarm

# Activate environment
source ~/setup_swarm_env.sh

# Install Python dependencies
pip install -r requirements.txt

# Generate drone models and world file for 3 drones
python scripts/generate_world.py --num-drones 3

# This creates:
#   - models/Drone1/, Drone2/, Drone3/ (from DroneTemplate)
#   - worlds/multi_drone_3.sdf
```

The `generate_world.py` script:
1. Creates drone models in `models/` by copying from `models/DroneTemplate/`
2. Each drone model gets a unique `fdm_port_in` for ArduPilot SITL communication
3. Generates a world file with all drones positioned in a line

You can generate more drones later:
```bash
python scripts/generate_world.py --num-drones 6 --spacing 8.0
```

---

## Step 9: Create ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Symlink swarm_ros package (don't copy - symlink allows easy development)
ln -s ~/swarm/swarm_ros swarm_ros

# Build the workspace
cd ~/ros2_ws
source ~/setup_swarm_env.sh
colcon build --packages-select swarm_ros

# Source the workspace (already in setup_swarm_env.sh for future sessions)
source install/setup.bash
```

### Build Troubleshooting

If you get Python path errors during build, ensure you're using the correct CMakeLists.txt. The key fix is using the versioned Python path:

```cmake
# In swarm_ros/CMakeLists.txt
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
```

This ensures Python modules are installed to `lib/python3.12/site-packages/` instead of `lib/python3/dist-packages/`, matching where ROS2 looks for them.

---

## Step 10: Verify Installation

### Test 1: Gazebo with Single Drone

```bash
# Terminal 1: Start Gazebo
source ~/setup_swarm_env.sh
gz sim -r ~/ardupilot_gazebo/worlds/iris_runway.sdf

# Terminal 2: Start SITL
source ~/setup_swarm_env.sh
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f JSON --console --out=udp:127.0.0.1:14540

# Wait for "Link 0 OK" in SITL console
# Drone should be visible in Gazebo
```

### Test 2: Multi-Drone Simulation

```bash
# Terminal 1: Start simulation (Gazebo + 3 SITL instances)
source ~/setup_swarm_env.sh
cd ~/swarm
python scripts/run_phase3_test.py --num-drones 3 --skip-test

# Wait for "SIMULATION READY" message

# Terminal 2: Start ROS2 bridge
source ~/setup_swarm_env.sh
ros2 launch swarm_ros simulation.launch.py num_drones:=3

# Terminal 3: Monitor swarm status
source ~/setup_swarm_env.sh
ros2 topic echo /swarm/status

# Terminal 4: Test formation flying
source ~/setup_swarm_env.sh

# Connect to drones
ros2 service call /swarm/connect std_srvs/srv/Trigger

# Arm and takeoff
ros2 service call /swarm/arm_all std_srvs/srv/Trigger
ros2 service call /swarm/takeoff_all std_srvs/srv/Trigger

# Fly V-formation
ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation \
  "{formation_type: 1, spacing: 5.0, altitude: 10.0, duration: 30.0}"
```

---

## ROS2 Topics and Services Reference

### Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/swarm/status` | SwarmStatus | 1Hz | Aggregated swarm status |
| `/drone_N/state` | DroneState | 10Hz | Per-drone position/health |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/swarm/connect` | Trigger | Connect to SITL drones |
| `/swarm/arm_all` | Trigger | Arm all drones |
| `/swarm/takeoff_all` | Trigger | Takeoff all drones |
| `/swarm/land_all` | Trigger | Land all drones |
| `/swarm/set_formation` | SetFormation | Set swarm formation |

### Formation Types

| ID | Name |
|----|------|
| 0 | LINE |
| 1 | V_FORMATION |
| 2 | TRIANGLE |
| 3 | GRID |
| 4 | CIRCLE |
| 5 | DIAMOND |

---

## Perception Pipeline (Phase 5)

The perception system adds YOLOv11-based object detection to the drone swarm.

### Prerequisites

```bash
# Install vision dependencies (large packages, ~2GB with PyTorch)
pip install ultralytics opencv-python

# Install ROS2 bridge packages
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image

# Rebuild ROS2 workspace to include perception messages
cd ~/ros2_ws
colcon build --packages-select swarm_ros
source install/setup.bash
```

### Test Detection Standalone

```bash
# Test YOLOv11 detector without simulation
source ~/setup_swarm_env.sh
cd ~/swarm
python scripts/test_phase5.py --detector-only
```

### Run Perception with Simulation

```bash
# Terminal 1: Start Gazebo with perception test world
source ~/setup_swarm_env.sh
cd ~/swarm
gz sim -r worlds/perception_test.sdf

# Terminal 2: Start SITL instances
python scripts/launch_sitl.py --num-instances 3

# Terminal 3: Start ROS2 bridge with cameras
source ~/setup_swarm_env.sh
ros2 launch swarm_ros simulation.launch.py num_drones:=3 enable_cameras:=true

# Terminal 4: Start perception nodes
source ~/setup_swarm_env.sh
ros2 launch swarm_ros perception.launch.py num_drones:=3

# Terminal 5: Monitor detections
source ~/setup_swarm_env.sh
ros2 topic echo /drone_0/detections
```

### Perception Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_N/camera/image` | sensor_msgs/Image | Camera feed from Gazebo |
| `/drone_N/detections` | DetectionArray | Detected objects with bounding boxes |

### Detection Classes

The system detects people and vehicles:
- Person (COCO class 0)
- Car (COCO class 2)
- Motorcycle (COCO class 3)
- Bus (COCO class 5)
- Truck (COCO class 7)

### Performance Notes

- GPU inference: ~10-20ms per frame (RTX series)
- CPU fallback: ~100-200ms per frame
- Camera resolution: 640x480 @ 15Hz
- Model: YOLOv11n (nano) by default

---

## Common Issues

### Issue: "ModuleNotFoundError: No module named 'swarm'"

**Cause:** PYTHONPATH doesn't include swarm project root.

**Fix:** Ensure `~/setup_swarm_env.sh` is sourced, which sets:
```bash
export PYTHONPATH="${SWARM_PROJECT_ROOT}:${PYTHONPATH}"
```

### Issue: "ModuleNotFoundError: No module named 'swarm_ros.swarm_bridge'"

**Cause:** Python modules installed to wrong path during colcon build.

**Fix:** Rebuild with the correct CMakeLists.txt that uses versioned Python path:
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select swarm_ros
source install/setup.bash
```

### Issue: Gazebo EGL/OpenGL errors on hybrid GPU laptops

**Cause:** Gazebo tries to use Intel GPU instead of NVIDIA.

**Fix:** Set NVIDIA environment variables (included in `setup_swarm_env.sh`):
```bash
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1
```

### Issue: "Unable to find or download file" in Gazebo

**Cause:** GZ_SIM_RESOURCE_PATH not set correctly.

**Fix:** Ensure ardupilot_gazebo paths are in GZ_SIM_RESOURCE_PATH:
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
```

### Issue: SITL shows "Link 1 down"

**Cause:** Gazebo not running or wrong frame type.

**Fix:**
1. Ensure Gazebo is running with `-r` flag (not paused)
2. Use `-f JSON` frame (not `-f gazebo-iris`)
3. Ensure the world file includes ArduPilotPlugin

### Issue: VSCode snap terminal library conflicts

**Cause:** VSCode snap injects conflicting glibc libraries.

**Fix:** Use external terminal (Ctrl+Alt+T) or clean LD_LIBRARY_PATH:
```bash
if [[ "$LD_LIBRARY_PATH" == *"snap/code"* ]]; then
    LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')
    export LD_LIBRARY_PATH
fi
```

### Issue: Gazebo version confusion (gazebo-classic vs gz-sim)

**Cause:** Two Gazebo ecosystems exist - deprecated `gazebo-classic` (versions 9, 11) and current `gz-sim` (Garden, Harmonic). Many old tutorials reference gazebo-classic.

**Fix:** Verify you have gz-sim:
```bash
gz sim --version  # Should show Harmonic or similar

# If you have gazebo-classic by mistake:
sudo apt remove gazebo*
# Then install gz-sim via official instructions
```

### Issue: MAVSDK waits forever on "Waiting to discover system"

**Cause:** By default, `sim_vehicle.py` only outputs MAVLink to port 14550. MAVSDK expects port 14540.

**Fix:** Add the `--out` flag:
```bash
sim_vehicle.py -v ArduCopter -f JSON --console --out=udp:127.0.0.1:14540
```

### Issue: Paths with spaces break Gazebo

**Cause:** Gazebo's resource path parsing doesn't handle spaces well.

**Fix:** Use a symlink:
```bash
ln -s "/path/with spaces/project" ~/swarm
```

### Issue: MAVProxy --out doesn't work with --no-mavproxy

**Cause:** The `--out` parameter is a MAVProxy feature. Without MAVProxy, there's nothing to forward.

**Fix:** Either use MAVProxy (remove `--no-mavproxy`), or connect directly to SITL's TCP port:
```python
# With MAVProxy: sim_vehicle.py -f JSON --out=udp:127.0.0.1:14540
system_address = "udpin://0.0.0.0:14540"

# Without MAVProxy: sim_vehicle.py -f JSON --no-mavproxy
system_address = "tcpout://127.0.0.1:5760"  # TCP port = 5760 + instance*10
```

### Issue: --speedup flag fails silently

**Cause:** SITL requires integer for `--speedup`.

**Fix:**
```bash
# WRONG
sim_vehicle.py -f JSON --speedup 1.0

# CORRECT
sim_vehicle.py -f JSON --speedup 1
```

### Issue: Multiple SITL instances conflict

**Cause:** Each SITL instance needs unique ports.

**Fix:** Use instance flag and unique output ports:
```bash
# Instance 0
sim_vehicle.py -v ArduCopter -f JSON -I0 --out=udp:127.0.0.1:14540

# Instance 1
sim_vehicle.py -v ArduCopter -f JSON -I1 --out=udp:127.0.0.1:14541

# Instance N
sim_vehicle.py -v ArduCopter -f JSON -I$N --out=udp:127.0.0.1:$((14540 + N))
```

### Issue: Performance problems with many drones

**Cause:** Gazebo + multiple SITL instances consume significant resources.

**Fix:**
- Use `--headless` for SITL when not debugging
- Reduce Gazebo physics rate: `<real_time_factor>0.5</real_time_factor>` in world file
- Run Gazebo server-only: `gz sim -s` (no GUI)
- Distribute SITL instances across CPU cores

---

## Project Structure

After installation, your directory structure should look like:

```
~/
├── ardupilot/                    # ArduPilot source (Copter-4.5)
├── ardupilot_gazebo/             # Gazebo plugin (DO NOT MODIFY)
│   ├── build/                    # Compiled plugin
│   ├── models/                   # Base models (iris_with_standoffs)
│   └── worlds/                   # Example world files
├── venv-ardupilot/               # Python virtual environment
├── ros2_ws/                      # ROS2 workspace
│   ├── src/
│   │   └── swarm_ros -> ~/swarm/swarm_ros  # Symlink
│   ├── build/
│   └── install/
├── swarm/                        # This project
│   ├── swarm/                    # Python package
│   │   ├── perception/           # Vision processing (Phase 5)
│   │   └── navigation/           # GPS-denied navigation (Phase 6)
│   ├── swarm_ros/                # ROS2 package
│   ├── models/                   # Drone models (Drone1, Drone2, etc.)
│   │   └── DroneTemplate/        # Base template with camera
│   ├── scripts/                  # Test and utility scripts
│   │   └── setup_env.sh          # Canonical environment setup
│   └── worlds/                   # Generated world files
└── setup_swarm_env.sh -> ~/swarm/scripts/setup_env.sh  # Symlink
```

**Important:** All drone models live in `~/swarm/models/`, NOT in `ardupilot_gazebo/`. The DroneTemplate includes an RGB camera sensor and is used to generate Drone1, Drone2, etc.

---

## Quick Reference Commands

```bash
# Activate environment (do this first in every terminal)
source ~/setup_swarm_env.sh

# Rebuild ROS2 workspace after changes
cd ~/ros2_ws && colcon build --packages-select swarm_ros && source install/setup.bash

# Start simulation (Gazebo + SITL)
python ~/swarm/scripts/run_phase3_test.py --num-drones 3 --skip-test

# Start ROS2 bridge
ros2 launch swarm_ros simulation.launch.py num_drones:=3

# Monitor topics
ros2 topic list
ros2 topic echo /swarm/status

# Call services
ros2 service call /swarm/connect std_srvs/srv/Trigger
ros2 service call /swarm/takeoff_all std_srvs/srv/Trigger
ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation "{formation_type: 1}"
ros2 service call /swarm/land_all std_srvs/srv/Trigger
```
