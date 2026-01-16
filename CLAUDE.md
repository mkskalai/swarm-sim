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
| [docker/README.md](docker/README.md) | Docker setup and usage |
| [docs/phase9-tactical-overwatch.md](docs/phase9-tactical-overwatch.md) | Phase 9 implementation plan |

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
| 6. GPS-Denied Nav | ✅ | `swarm/navigation/`, ORB+optical flow VO, EKF fusion |
| 7. Autonomous Missions | ✅ | `swarm/missions/`, `swarm/coordination/search_patterns.py`, `pursuit_controller.py` |
| 8. Docker | ✅ | `docker/Dockerfile`, `docker-compose.yml` |
| 9. Tactical Overwatch | ⬜ | `swarm/tactical/` - Coverage mesh, threat intercept, observer interface |

### Phase 7 Details: Autonomous Missions

**Status: COMPLETE**

**Components Implemented:**
- **Terrain worlds:** Urban, forest, canyon Jinja2 templates (`worlds/terrain.sdf.jinja`)
- **GPS jamming simulator:** Zone-based + probabilistic denial (`swarm/navigation/gps_jammer.py`)
- **Pursuit behavior:** Multi-drone tracking with FOLLOW/INTERCEPT/SURROUND/SHADOW strategies
- **Search patterns:** Lawnmower, spiral, expanding square with dynamic reallocation
- **Semantic landmarks:** YOLO detections as VIO position corrections

**Key files:**
- `swarm/coordination/search_patterns.py` - SearchPatternGenerator, AdaptiveSearchController
- `swarm/coordination/pursuit_controller.py` - PursuitController, SimulatedTarget
- `swarm/navigation/gps_jammer.py` - GPSJammer, GPSDenialZone
- `swarm/navigation/semantic_landmarks.py` - SemanticLandmarkDatabase
- `swarm/missions/autonomous_mission.py` - AutonomousMissionController
- `swarm/missions/terrain_config.py` - TerrainConfig, terrain generators

**Test scenarios:**
1. Swarm navigates terrain → 30-50% get jammed → jammed drones switch to VIO + P2P relative positioning
2. Pursuit through urban canyon with intermittent GPS denial
3. Search pattern maintains coverage when subset loses GPS

**Run tests:** `python scripts/test_phase7.py --scenario standalone`

### Phase 8 Details: Docker Containerization

**Status: COMPLETE**

**Components Implemented:**
- **Multi-stage Dockerfile:** Base → ArduPilot builder → Gazebo plugin builder → Final image
- **docker-compose.yml:** Profiles for dev, ci, and headless modes
- **GPU support:** NVIDIA with automatic CPU fallback
- **GUI support:** X11 forwarding for Gazebo visualization
- **Helper scripts:** `build.sh`, `run.sh` for easy usage

**Key files:**
- `docker/Dockerfile` - Multi-stage build (~15-20GB final image)
- `docker/docker-compose.yml` - Orchestration with dev/ci/headless profiles
- `docker/entrypoint.sh` - Environment setup and GPU detection
- `docker/scripts/build.sh` - Build helper
- `docker/scripts/run.sh` - Run helper with X11 setup
- `docker/README.md` - Docker usage documentation

**Usage modes:**
1. **Development:** Mount source code for live editing
2. **CI/CD:** Self-contained testing with pytest
3. **Headless:** Server-side simulation without GUI

**Run container:**
```bash
cd docker
./scripts/build.sh          # Build image (30-45 min first time)
./scripts/run.sh dev        # Development mode
./scripts/run.sh sim 3      # Run 3-drone simulation
./scripts/run.sh ci         # Run tests
```

### Phase 9 Details: Tactical Swarm Overwatch

**Status: PLANNED** - See [docs/phase9-tactical-overwatch.md](docs/phase9-tactical-overwatch.md) for full plan.

**Mission:** Protective drone coverage over human group navigating labyrinth with threat interception.

**Components Planned:**
- **Group tracking:** GPS beacons from group members + visual fallback + observer input
- **Coverage mesh:** Hexagonal formation dynamically following group
- **Relative positioning:** Formation maintenance during GPS denial using anchor drones
- **Threat detection:** YOLO-based enemy drone detection + classification (FRIENDLY/HOSTILE)
- **Intercept planning:** Collision course computation for kamikaze defense
- **Defender selection:** Weighted selection (prefer GPS-denied, low battery, edge drones)
- **Observer interface:** CLI + ROS2 commands for human-swarm control

**Key decisions:**
- Always intercept threats (no minimum drone threshold)
- Sacrifice GPS-denied and low-battery drones first
- Human observers direct high-level behavior, swarm handles tactics

**Reuses from existing phases:**
- VIO/EKF (Phase 6) for GPS-denied operation
- GPS Jammer (Phase 7) for jamming zone simulation
- YOLO detector (Phase 5) with new drone class
- NeighborTracker (Phase 4) for swarm awareness
- PursuitController concepts (Phase 7) for intercept

**Implementation order:**
1. GroupTracker → 2. CoverageMesh → 3. RelativePositioning → 4. DroneDataset → 5. ThreatDetector → 6. InterceptPlanner → 7. DefenderSelector → 8. ObserverInterface → 9. MissionController → 10. TestScenarios

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

### 7. VIO Must Initialize After Takeoff
VIO needs an initial GPS position for scale estimation. Call `init_vio_all()` after `takeoff_all()`, not before. The altitude at initialization is used for monocular scale.

### 8. Monocular Scale Drift
Without stereo cameras, VIO scale drifts over time. In hybrid mode, GPS automatically corrects scale. In VIO-only mode, expect gradual position drift.

### 9. ROS2 Nodes Can't Find `swarm` Module
`colcon build` **copies** Python files to `~/ros2_ws/install/`, breaking symlink context. ROS2 nodes need explicit path resolution to find the `swarm` module. See `vio_node.py` for the multi-method approach (env var → symlink → hardcoded path).

---

## Quick Commands

```bash
# Create symlink if not exists (one-time setup)
ln -sf ~/swarm/scripts/setup_env.sh ~/setup_swarm_env.sh

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

# Phase 6: GPS-denied navigation test
python scripts/test_phase6.py --num-drones 3 --gps-denial-test

# Phase 7: Autonomous missions test (standalone - no sim required)
python scripts/test_phase7.py --scenario standalone

# Phase 7: Generate terrain world
python scripts/generate_terrain_world.py --terrain urban --num-drones 4

# Run unit tests
pytest tests/test_navigation.py -v
pytest tests/test_search_patterns.py -v
pytest tests/test_pursuit.py -v
pytest tests/test_gps_jammer.py -v

# Docker: Build and run
cd docker && ./scripts/build.sh
./scripts/run.sh dev         # Development mode with source mounts
./scripts/run.sh sim 3       # Run 3-drone simulation
./scripts/run.sh ci          # Run CI tests
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
| `VIOEstimator` | `swarm/navigation/vio_estimator.py` | Visual-Inertial Odometry orchestrator |
| `NavigationEKF` | `swarm/navigation/ekf.py` | Extended Kalman Filter for sensor fusion |
| `VisualOdometry` | `swarm/navigation/visual_odometry.py` | ORB + optical flow VO |
| `PositionSource` | `swarm/navigation/position_source.py` | Hybrid GPS/VIO position provider |
| `SearchPatternGenerator` | `swarm/coordination/search_patterns.py` | Lawnmower, spiral, expanding square patterns |
| `AdaptiveSearchController` | `swarm/coordination/search_patterns.py` | Distributed search with failure reallocation |
| `PursuitController` | `swarm/coordination/pursuit_controller.py` | Multi-drone target tracking |
| `GPSJammer` | `swarm/navigation/gps_jammer.py` | Zone-based GPS denial simulation |
| `SemanticLandmarkDatabase` | `swarm/navigation/semantic_landmarks.py` | YOLO detection landmarks for VIO |
| `AutonomousMissionController` | `swarm/missions/autonomous_mission.py` | Mission orchestrator (search→pursue→RTL) |
| `TacticalMissionController` | `swarm/tactical/mission_controller.py` | Phase 9: Tactical overwatch orchestrator (planned) |
| `GroupTracker` | `swarm/tactical/group_tracker.py` | Phase 9: Track protected human group (planned) |
| `CoverageMeshPlanner` | `swarm/tactical/coverage_mesh.py` | Phase 9: Dynamic formation over group (planned) |
| `ThreatDetector` | `swarm/tactical/threat_detector.py` | Phase 9: Enemy drone detection (planned) |
| `InterceptPlanner` | `swarm/tactical/intercept_planner.py` | Phase 9: Collision course computation (planned) |
| `DefenderSelector` | `swarm/tactical/defender_selector.py` | Phase 9: Choose drone for intercept (planned) |
| `ObserverInterface` | `swarm/tactical/observer_interface.py` | Phase 9: Human-swarm commands (planned) |
| `RelativePositioning` | `swarm/tactical/relative_positioning.py` | Phase 9: GPS-denied formation keeping (planned) |

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
| 2026-01-16 | ORB+optical flow + EKF | ORB primary, flow fallback, error-state EKF for VIO |
| 2026-01-16 | Zone-based GPS jamming | Geographic zones + probability for realistic denial simulation |
| 2026-01-16 | Adaptive search patterns | Dynamic reallocation when drones fail during search |
| 2026-01-16 | Surround pursuit strategy | Multi-drone encirclement default; intercept for fast targets |
| 2026-01-16 | Phase 9: Tactical Overwatch | Human-directed swarm for labyrinth protection scenario |
| 2026-01-16 | GPS beacon + visual hybrid | Group tracking robust to jamming zones |
| 2026-01-16 | Always intercept policy | No minimum drone threshold; neutralize all threats |
| 2026-01-16 | Sacrifice jammed/low-battery | Defender selection prefers expendable drones |
| 2026-01-16 | CLI + ROS2 observer interface | Start simple, extensible to web dashboard later |
| 2026-01-16 | Single all-in-one Docker image | Simpler deployment; multi-container adds complexity for little benefit |
| 2026-01-16 | Multi-stage Docker build | Reduces final image size; separates build deps from runtime |
| 2026-01-16 | Host network mode | Simplifies MAVLink port exposure (many dynamic ports) |
