# Docker Setup for Drone Swarm Simulation

This directory contains Docker configuration for running the drone swarm simulation platform in a container.

## Prerequisites

### Required
- Docker Engine 24.0+
- Docker Compose v2.20+

### Optional (for GPU acceleration)
- NVIDIA GPU with driver 525+
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

### For GUI (Gazebo visualization)
- X11 display server (Linux)
- On macOS: XQuartz
- On Windows: VcXsrv or similar

## Quick Start

### 1. Build the Image

```bash
cd docker
./scripts/build.sh
```

First build takes 30-45 minutes (ArduPilot compilation). Subsequent builds use cached layers.

### 2. Run

**Interactive shell:**
```bash
./scripts/run.sh
```

**Development mode** (mounts source code for live editing):
```bash
./scripts/run.sh dev
```

**Run N-drone simulation:**
```bash
./scripts/run.sh sim 3
```

**Run tests:**
```bash
./scripts/run.sh ci
```

## Usage Modes

### Development Mode (`dev`)

Mounts local source code into the container for live editing:
- `./swarm/` → `/app/swarm/`
- `./models/` → `/app/models/`
- `./worlds/` → `/app/worlds/`
- `./scripts/` → `/app/scripts/`
- `./logs/` → `/app/logs/`

```bash
./scripts/run.sh dev

# Inside container:
python scripts/run_phase3_test.py --num-drones 3 --skip-test
```

### CI/CD Mode (`ci`)

Self-contained test execution with no external dependencies:

```bash
./scripts/run.sh ci
```

Or run specific tests:
```bash
./scripts/run.sh test tests/test_navigation.py
```

### Headless Mode (`headless`)

Run simulation without GUI (for servers):

```bash
./scripts/run.sh headless
```

## GPU Support

The container automatically detects NVIDIA GPUs and uses them for:
- Gazebo rendering
- YOLO inference (10x faster than CPU)

If no GPU is available, it gracefully falls back to CPU.

### Verify GPU access:

```bash
docker compose -f docker/docker-compose.yml run --rm swarm nvidia-smi
```

### Check YOLO GPU:

```bash
docker compose -f docker/docker-compose.yml run --rm swarm \
  python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

## GUI (Gazebo Visualization)

### Linux

X11 forwarding is set up automatically. Just run:

```bash
./scripts/run.sh dev
# Then inside container:
gz sim -r /app/worlds/multi_drone_3.sdf
```

If you get display errors, run manually:
```bash
xhost +local:docker
```

### macOS

1. Install [XQuartz](https://www.xquartz.org/)
2. Open XQuartz → Preferences → Security → "Allow connections from network clients"
3. Restart XQuartz
4. Run:
```bash
xhost +localhost
export DISPLAY=host.docker.internal:0
./scripts/run.sh dev
```

### Windows (WSL2)

1. Install VcXsrv or X410
2. Set `DISPLAY=:0` in WSL2
3. Run the container

## Manual Docker Commands

**Build:**
```bash
docker compose -f docker/docker-compose.yml build
```

**Interactive shell:**
```bash
docker compose -f docker/docker-compose.yml run --rm swarm bash
```

**Run simulation:**
```bash
docker compose -f docker/docker-compose.yml run --rm swarm \
  python scripts/run_phase3_test.py --num-drones 3
```

**ROS2 commands:**
```bash
docker compose -f docker/docker-compose.yml run --rm swarm \
  ros2 topic list
```

## Troubleshooting

### "permission denied" on X11 socket

```bash
xhost +local:docker
```

### NVIDIA driver mismatch

Ensure host driver matches container CUDA version:
```bash
nvidia-smi  # Check host driver version
```

The container uses CUDA 12.1. If your driver is older, rebuild with a different PyTorch version.

### Out of disk space

The full image is ~15-20GB. Clean up old images:
```bash
docker system prune -a
```

### ArduPilot SITL won't start

Check if ports are already in use:
```bash
lsof -i :14540
lsof -i :9002
```

### Gazebo crashes with EGL error

Try forcing software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

Or ensure NVIDIA EGL is configured:
```bash
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

## Container Architecture

```
/app/                    # Working directory
├── swarm/               # Python swarm package
├── swarm_ros/           # ROS2 package source
├── ros2_ws/             # ROS2 workspace (built)
├── models/              # Gazebo drone models
├── worlds/              # Gazebo world files
├── scripts/             # Simulation scripts
├── tests/               # Test suite
└── logs/                # Runtime logs

/opt/ardupilot/          # ArduPilot SITL
/opt/ardupilot_gazebo/   # Gazebo plugin
```

## Environment Variables

The container sets these automatically:
- `GZ_SIM_SYSTEM_PLUGIN_PATH` - Gazebo plugin path
- `GZ_SIM_RESOURCE_PATH` - Gazebo model/world paths
- `PYTHONPATH` - Includes `/app` for swarm module
- `PATH` - Includes ArduPilot tools

## Networking

The container uses `network_mode: host` to simplify MAVLink port exposure:
- FDM ports: 9002, 9012, 9022, ... (Gazebo ↔ SITL)
- MAVLink ports: 14540, 14541, 14542, ... (pymavlink)

## Image Size

| Component | Size |
|-----------|------|
| Ubuntu 24.04 base | ~80MB |
| ROS2 Jazzy Desktop | ~3GB |
| Gazebo Harmonic | ~2GB |
| ArduPilot SITL | ~1GB |
| ardupilot_gazebo | ~100MB |
| Python + PyTorch/YOLO | ~5GB |
| **Total** | **~12-15GB** |
