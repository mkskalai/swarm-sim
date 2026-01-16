# Drone Swarm Simulation Platform

> AI/Developer context file. For setup see [INSTALL.md](INSTALL.md). For usage see [docs/user-guide.md](docs/user-guide.md).

## Project Overview

Drone swarm simulation using ArduPilot SITL, Gazebo Harmonic, ROS2 Jazzy. Target: 6-15 quadcopters with RGB cameras for surveillance/reconnaissance research.

## Documentation Index

| Document | Purpose |
|----------|---------|
| [INSTALL.md](INSTALL.md) | Installation guide + troubleshooting |
| [docs/architecture.md](docs/architecture.md) | System design, component diagrams |
| [docs/user-guide.md](docs/user-guide.md) | How to run simulations |
| [docs/api/](docs/api/) | API reference (core, coordination, perception, ros2) |
| [docs/contributing.md](docs/contributing.md) | Development workflow |

---

## Architecture Decisions

| Decision | Choice | Why |
|----------|--------|-----|
| MAVLink library | **pymavlink** | MAVSDK routes by sysid (all SITL have sysid=1) - fails for multi-drone. pymavlink uses port isolation. |
| Control | **Hybrid** | Centralized GCS + decentralized P2P. Pure centralized fails on disconnect; pure P2P is complex. |
| Gazebo-SITL | **JSON interface** | ardupilot_gazebo uses JSON, not legacy FDM protocol. Use `-f JSON` not `-f gazebo-iris`. |
| ROS2 integration | **SwarmBridge** | Wraps existing SwarmController for simulation. DroneNode for real hardware. |
| Detection | **YOLOv11** | Modern, pip-installable, GPU with CPU fallback. |

## Port Scheme

| Drone | FDM Port (Gazebo↔SITL) | MAVLink Port |
|-------|------------------------|--------------|
| Drone1 | 9002 | 14540 |
| Drone2 | 9012 | 14541 |
| Drone3 | 9022 | 14542 |
| DroneN | 9002 + (N-1)*10 | 14540 + (N-1) |

---

## Development Phases

| Phase | Status | Key Files |
|-------|--------|-----------|
| 1. Single Drone | ✅ | `swarm/core/drone.py` |
| 2. Multi-Drone | ✅ | `swarm/core/fleet.py`, `scripts/generate_world.py` |
| 3. Coordination | ✅ | `swarm/coordination/swarm_controller.py`, `formations.py` |
| 4. ROS2 | ✅ | `swarm_ros/swarm_ros/swarm_bridge.py` |
| 5. Perception | ✅ | `swarm/perception/detector.py`, `tracker.py` |
| 6. GPS-Denied Nav | ⬜ | Visual odometry, IMU fusion |
| 7. Behaviors | ⬜ | Pursuit, search patterns |
| 8. Docker | ⬜ | Containerization |

---

## Key Development Gotchas

These are code-level issues. For setup issues see [INSTALL.md](INSTALL.md#common-issues).

### 1. EKF Must Converge Before Arming
ArduPilot rejects arming until EKF converges (~30-60s). Always call `wait_for_ekf_all()` before `arm_all()`.

### 2. Nested Model Includes Break Camera Joints
Gazebo joints cannot reference links inside `<include>` models. Camera ends up detached. **Solution:** Create standalone models (copy full content, don't nest includes).

### 3. Camera Topics Must Be Unique
Multiple drones with `<topic>camera</topic>` all publish to same global topic. Use `<topic>drone_N/camera</topic>`.

### 4. Formation Commands Must Wait for Arrival
Always verify drones reached positions before continuing. The `fly_formation()` method handles this with `arrival_timeout` and `arrival_tolerance` parameters.

### 5. ROS2 Python Module Path
CMakeLists.txt must use versioned path: `lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/`

### 6. Always Log SITL Output
Never use `subprocess.DEVNULL` for SITL - silent failures are impossible to debug. Logs go to `logs/sitl_N_stdout.log`.

---

## Quick Commands

```bash
# Activate environment
source ~/setup_swarm_env.sh

# Run 3-drone test
python scripts/run_phase3_test.py --num-drones 3

# Start sim only (for manual control)
python scripts/run_phase3_test.py --num-drones 3 --skip-test

# ROS2 bridge
ros2 launch swarm_ros simulation.launch.py num_drones:=3

# Rebuild ROS2 after changes
cd ~/ros2_ws && colcon build --packages-select swarm_ros && source install/setup.bash
```

---

## Key Classes

| Class | Location | Purpose |
|-------|----------|---------|
| `SwarmController` | `swarm/coordination/swarm_controller.py` | Main orchestrator (pymavlink) |
| `FormationType` | `swarm/coordination/formations.py` | LINE, V, TRIANGLE, GRID, CIRCLE, DIAMOND |
| `YOLODetector` | `swarm/perception/detector.py` | Object detection |
| `SimpleTracker` | `swarm/perception/tracker.py` | IoU-based tracking |
| `SwarmBridge` | `swarm_ros/swarm_ros/swarm_bridge.py` | ROS2 interface for simulation |

---

## Decisions Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2025-01-12 | Hybrid architecture | Balance coordination and resilience |
| 2025-01-14 | pymavlink over MAVSDK | Port isolation for multi-drone |
| 2025-01-14 | fdm_port_out required | Necessary despite being "deprecated" |
| 2026-01-14 | SwarmBridge pattern | Preserve working pymavlink code |
| 2026-01-14 | YOLOv11 + IoU tracker | Lightweight, sufficient for surveillance |
| 2026-01-14 | Standalone drone models | Nested includes break camera joints |
