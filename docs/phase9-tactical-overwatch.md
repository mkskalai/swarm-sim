# Phase 9: Tactical Swarm Overwatch

> Implementation plan for protective drone coverage over human groups with threat interception capabilities.

**Status:** PLANNED
**Dependencies:** Phases 1-7 (multi-drone control, formations, perception, VIO, GPS denial, pursuit)
**Estimated new code:** ~2,900 lines
**Reused from existing:** ~1,050 lines

---

## Mission Overview

A drone swarm provides protective aerial coverage over a human group navigating through a labyrinth environment. The system features:

- **Dynamic coverage mesh** that follows the group
- **Human observers** who direct drones via commands
- **Threat detection** identifying enemy drones
- **Kamikaze interception** to neutralize threats
- **GPS-denied operation** through jamming zones

```
                    ┌─────────────────────────────────────┐
                    │         LABYRINTH (top view)        │
                    │  ┌───┐     ┌───────┐    ┌────┐     │
                    │  │   │     │       │    │    │     │
    Enemy Drone ──▶ │  │   ╠═════╣   G   ╠════╣    │     │
        X           │  │   │     │  ***  │    │    │     │
                    │  │   │     │  **   │    └────┘     │
                    │  └───┘     └───────┘               │
   ░░░░░░░░░        │            ^ ^ ^ ^                 │
   ░JAMMING░        │            D D D D  (drone mesh)   │
   ░░░░░░░░░        │                                    │
                    └─────────────────────────────────────┘

    G = Group of people (protected)
    * = Sky observers (looking up, directing drones)
    D = Friendly drones (coverage mesh)
    X = Enemy drone (threat to be intercepted)
```

---

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      TACTICAL MISSION CONTROLLER                        │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐  ┌──────────────┐  │
│  │  Observer   │  │    Group     │  │   Threat    │  │   Defender   │  │
│  │  Interface  │  │   Tracker    │  │  Detector   │  │   Selector   │  │
│  └──────┬──────┘  └──────┬───────┘  └──────┬──────┘  └──────┬───────┘  │
│         │                │                 │                │          │
│         ▼                ▼                 ▼                ▼          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                      COVERAGE MESH PLANNER                      │   │
│  │   - Dynamic formation centered on group                         │   │
│  │   - Maintains spacing, adapts to losses                         │   │
│  │   - Handles GPS-denied drones via relative positioning          │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    │                                    │
│         ┌──────────────────────────┼──────────────────────────┐        │
│         ▼                          ▼                          ▼        │
│  ┌─────────────┐           ┌─────────────┐           ┌─────────────┐   │
│  │  INTERCEPT  │           │   COVER     │           │    COVER    │   │
│  │    MODE     │           │    MODE     │           │  (VIO mode) │   │
│  │  (kamikaze) │           │  (formation)│           │  GPS-denied │   │
│  └─────────────┘           └─────────────┘           └─────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │   EXISTING SWARM CONTROLLER   │
                    │      (pymavlink commands)     │
                    └───────────────────────────────┘
```

### State Machine

```
┌──────────────┐
│ INITIALIZING │
└──────┬───────┘
       │ all drones ready
       ▼
┌──────────────┐◄─────────────────────────────────┐
│   COVERING   │                                  │
└──────┬───────┘                                  │
       │ threat detected                          │
       ▼                                          │
┌──────────────┐  threat departed    ┌────────────┴───┐
│   THREAT     │────────────────────►│                │
│  DETECTED    │                     │                │
└──────┬───────┘                     │                │
       │ defender selected           │                │
       ▼                             │                │
┌──────────────┐                     │                │
│ INTERCEPTING │                     │                │
└──────┬───────┘                     │                │
       │ intercept complete          │                │
       ▼                             │                │
┌──────────────┐  mesh reformed      │                │
│  REGROUPING  │─────────────────────┘                │
└──────────────┘                                      │
                                                      │
┌──────────────┐  observer command / critical failure │
│  EMERGENCY   │◄─────────────────────────────────────┘
└──────┬───────┘
       │
       ▼
┌──────────────┐
│     RTL      │
└──────────────┘
```

---

## Component Specifications

### 9.1 Group Tracker

**Purpose:** Track the position of the protected human group through the labyrinth.

**File:** `swarm/tactical/group_tracker.py`

#### Inputs

| Source | Description | Reliability |
|--------|-------------|-------------|
| GPS beacons | Group members carry GPS transmitters | High (when not jammed) |
| Visual detection | YOLO person detection from drones | Medium |
| Observer input | Manual position updates | High (but infrequent) |

#### Outputs

| Output | Type | Description |
|--------|------|-------------|
| `group_position` | PositionNED | Centroid of the group |
| `group_velocity` | VelocityNED | Group movement vector |
| `group_bounds` | BoundingBox | Spatial extent of group |
| `confidence` | float | Tracking confidence (0-1) |

#### Logic

```python
class GroupTracker:
    """Tracks the protected human group."""

    class TrackingMethod(Enum):
        GPS_BEACONS = "gps"           # Primary: GPS from group members
        VISUAL_DETECTION = "visual"    # Fallback: YOLO person detection
        OBSERVER_INPUT = "manual"      # Override: Manual from observers
        HYBRID = "hybrid"              # Fusion of all sources

    def update_from_gps_beacon(self, member_id: str, position: PositionNED):
        """Update from GPS carried by group member."""

    def update_from_detection(self, drone_id: int, detections: List[Detection]):
        """Update from drone's downward camera detecting people."""

    def update_from_observer(self, position: PositionNED, bounds: BoundingBox):
        """Update from sky observer's manual input."""

    def get_coverage_center(self) -> PositionNED:
        """Get optimal center point for drone coverage mesh."""

    def get_coverage_radius(self) -> float:
        """Get radius needed to cover entire group."""

    def predict_position(self, dt: float) -> PositionNED:
        """Predict group position dt seconds in future."""
```

#### Fusion Strategy

1. **Primary:** GPS beacons from group members (when available)
2. **Fallback:** Visual detection when GPS jammed
3. **Override:** Observer manual input (highest priority)
4. **Fusion:** Weighted average based on source age and reliability

---

### 9.2 Coverage Mesh Planner

**Purpose:** Compute and maintain drone positions in a protective formation over the group.

**File:** `swarm/tactical/coverage_mesh.py`

#### Mesh Patterns

```
HEXAGONAL (default):       GRID:                PERIMETER:
    D   D   D              D   D   D              D   D
      D   D                                     D       D
    D   D   D              D   D   D            D   G   D
      [G]                      [G]              D       D
    D   D   D              D   D   D              D   D
      D   D
    D   D   D              D   D   D

[G] = Group center, D = Drone position
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pattern` | HEXAGONAL | Formation pattern type |
| `base_altitude` | 20.0m | Height above group |
| `drone_spacing` | 15.0m | Distance between drones |
| `coverage_margin` | 1.2 | Multiplier on group radius |

#### Key Methods

```python
class CoverageMeshPlanner:
    def compute_mesh(
        self,
        group_center: PositionNED,
        group_radius: float,
        available_drones: List[int],
    ) -> Dict[int, PositionNED]:
        """Compute target positions for all available drones."""

    def on_drone_detached(self, drone_id: int, reason: str):
        """Handle drone leaving mesh (intercept, failure)."""
        # Recompute mesh with remaining drones

    def on_drone_returned(self, drone_id: int):
        """Handle drone rejoining mesh."""

    def get_optimal_interceptor(
        self,
        threat_position: PositionNED,
        threat_velocity: VelocityNED,
    ) -> int:
        """Select best drone to detach for intercept."""
```

#### Behavior

- Formation automatically re-centers as group moves
- When a drone is detached for intercept, remaining drones spread to maintain coverage
- Supports dynamic scaling (more drones = tighter mesh, fewer = wider spacing)

---

### 9.3 Relative Positioning

**Purpose:** Maintain formation when drones lose GPS in jamming zones.

**File:** `swarm/tactical/relative_positioning.py`

#### Concept

```
Normal (GPS):              GPS-Denied (Relative):

D1[GPS] ─── D2[GPS]        D1[GPS] ─── D2[GPS]    (anchors)
   │           │              │     ╲   ╱  │
   │           │              │      ╲ ╱   │
D3[GPS] ─── D4[GPS]        D3[VIO]  ───  D4[VIO]  (relative to anchors)

Anchor = drone with GPS
Jammed = drone using VIO + relative position to anchor
```

#### Logic

```python
class RelativePositioning:
    def __init__(self, neighbor_tracker: NeighborTracker):
        self.neighbors = neighbor_tracker
        self._anchor_drones: Set[int] = set()  # Drones with GPS

    def update_anchors(self, drones_with_gps: Set[int]):
        """Update which drones can serve as position anchors."""

    def compute_relative_target(
        self,
        drone_id: int,
        absolute_target: PositionNED,
        mesh_positions: Dict[int, PositionNED],
    ) -> RelativePosition:
        """Convert absolute target to relative (for GPS-denied drone)."""
        # Find nearest anchor, compute offset

    def get_position_estimate(
        self,
        drone_id: int,
        vio_position: PositionNED,
    ) -> PositionNED:
        """Get best position estimate for GPS-denied drone."""
        # Fuse VIO + relative measurements
```

#### Depends On

- Existing `VIOEstimator` (Phase 6)
- Existing `NeighborTracker` (Phase 4)
- Existing `GPSJammer` (Phase 7)

---

### 9.4 Threat Detector

**Purpose:** Detect, track, and classify airborne threats.

**File:** `swarm/tactical/threat_detector.py`

#### Classification

| Classification | Criteria | Action |
|---------------|----------|--------|
| FRIENDLY | Matches known swarm position | Ignore |
| UNKNOWN | Unidentified flying object | Monitor |
| HOSTILE | Approaching group, drone-like | Intercept |
| BIRD | Erratic motion, small, no threat vector | Ignore |

#### Threat Scoring

```python
@dataclass
class Threat:
    track_id: int
    classification: ThreatClassification
    position: PositionNED
    velocity: VelocityNED
    confidence: float

    # Computed fields
    closest_approach_to_group: float      # Meters
    time_to_closest_approach: float       # Seconds
    threat_level: float                   # 0-1 priority score
```

**Threat level formula:**
```
threat_level = (1 / closest_approach) * (1 / time_to_approach) * confidence * hostile_weight
```

#### Pipeline

```
Camera frames → YOLO detect → Track over time → Classify → Score → Priority queue
      │              │              │              │            │
      │              │              │              │            └─► Intercept decision
      │              │              │              └─► Match against friendly positions
      │              │              └─► SimpleTracker (existing)
      │              └─► "drone" class (requires training)
      └─► All drones contribute detections (fused)
```

#### Depends On

- Existing `YOLODetector` (Phase 5)
- Existing `SimpleTracker` (Phase 5)
- Existing `NeighborTracker` (Phase 4) - to identify friendlies

---

### 9.4b Drone Detection Dataset

**Purpose:** Train YOLO to detect drone class.

**File:** `scripts/generate_drone_dataset.py`

#### Approach

1. Add drone 3D models to Gazebo (enemy drone variants)
2. Spawn at various positions, angles, distances
3. Capture images from swarm drone cameras
4. Auto-generate YOLO labels from ground truth positions
5. Augment with lighting, blur, noise variations
6. Train YOLOv11 with new class

#### Deliverables

```
data/drone_detection/
├── images/
│   ├── train/           # ~2000 synthetic images
│   └── val/             # ~500 validation images
├── labels/
│   ├── train/           # YOLO format labels
│   └── val/
└── weights/
    └── drone_yolov11.pt # Trained weights
```

---

### 9.5 Intercept Planner

**Purpose:** Compute collision trajectories for threat neutralization.

**File:** `swarm/tactical/intercept_planner.py`

#### Intercept Geometry

```
                    Threat
                      ●───────────► threat_velocity
                     ╱
                    ╱ intercept
                   ╱  trajectory
                  ╱
                 ╱
    Interceptor ●
                 ╲
                  ╲───────────► current_velocity

    Goal: Compute velocity vector so interceptor reaches
          same point as threat at same time
```

#### Algorithm (Lead Pursuit)

```python
def compute_intercept(
    interceptor_pos: PositionNED,
    target_pos: PositionNED,
    target_vel: VelocityNED,
    max_speed: float,
) -> Optional[InterceptSolution]:
    """
    Solve: interceptor_pos + v*t = target_pos + target_vel*t

    This is a quadratic in t:
    |v|^2 * t^2 - 2*(rel_pos · target_vel)*t - |rel_pos|^2 = 0

    Where |v| = max_speed (interceptor flies at max)
    """
    rel_pos = target_pos - interceptor_pos

    # Quadratic coefficients
    a = max_speed**2 - np.dot(target_vel, target_vel)
    b = -2 * np.dot(rel_pos, target_vel)
    c = -np.dot(rel_pos, rel_pos)

    # Solve for time
    discriminant = b**2 - 4*a*c
    if discriminant < 0:
        return None  # Can't intercept (target too fast)

    t = (-b + sqrt(discriminant)) / (2*a)
    if t < 0:
        return None  # Intercept in past

    # Intercept point
    intercept_point = target_pos + target_vel * t

    # Required velocity
    intercept_vel = (intercept_point - interceptor_pos) / t

    return InterceptSolution(
        intercept_point=intercept_point,
        time_to_intercept=t,
        intercept_velocity=intercept_vel,
    )
```

#### Output

```python
@dataclass
class InterceptSolution:
    interceptor_id: int
    target_track_id: int
    intercept_point: PositionNED          # Where collision occurs
    time_to_intercept: float              # Seconds
    intercept_velocity: VelocityNED       # Required velocity
    success_probability: float            # Based on uncertainties
```

---

### 9.6 Defender Selector

**Purpose:** Choose which drone performs the intercept.

**File:** `swarm/tactical/defender_selector.py`

#### Selection Criteria

| Factor | Weight | Preference |
|--------|--------|------------|
| Intercept time | 0.30 | Lower is better (faster response) |
| Success probability | 0.30 | Higher is better |
| Mesh criticality | 0.20 | Lower is better (expendable position) |
| GPS status | 0.10 | Prefer GPS-denied (sacrifice jammed drones) |
| Battery level | 0.10 | Prefer low battery (near death anyway) |

#### Mesh Criticality

```
Low criticality (edges):     High criticality (center):

    D ← expendable              D
     ╲                           ╲
      D                           D ← critical
       ╲                           ╲
        [G]                         [G]
```

Edge drones and those with redundant coverage are preferred for intercept.

#### Policy

**Always intercept** - No minimum drone threshold. If a threat is hostile and interceptable, send a defender.

---

### 9.7 Observer Interface

**Purpose:** Human-swarm command and feedback interface.

**Files:**
- `swarm/tactical/observer_interface.py` - Core logic
- `swarm_ros/swarm_ros/observer_node.py` - ROS2 wrapper
- `scripts/observer_cli.py` - CLI tool

#### Commands

| Category | Command | Parameters | Description |
|----------|---------|------------|-------------|
| Coverage | MOVE_COVERAGE | position | Shift coverage center |
| Coverage | EXPAND_COVERAGE | factor | Increase coverage radius |
| Coverage | CONTRACT_COVERAGE | factor | Decrease coverage radius |
| Coverage | SET_ALTITUDE | meters | Change mesh altitude |
| Threat | MARK_HOSTILE | track_id | Force classification as hostile |
| Threat | MARK_FRIENDLY | track_id | Force classification as friendly |
| Threat | INTERCEPT_NOW | track_id | Immediate intercept |
| Threat | CANCEL_INTERCEPT | drone_id | Abort intercept |
| Group | UPDATE_GROUP_POS | position | Manual group position |
| Emergency | RTL_ALL | - | Return all to launch |
| Emergency | HOLD_POSITION | - | Freeze all movement |

#### Feedback

| Topic | Content | Rate |
|-------|---------|------|
| `/swarm/status` | Drone positions, states, GPS status | 2 Hz |
| `/swarm/threats` | Active threat list | On change |
| `/swarm/intercepts` | Active intercept status | On change |
| `/swarm/coverage` | Coverage quality metrics | 1 Hz |

#### Implementation Phases

**Phase 9.7a (MVP):** CLI + ROS2 topics
- Command via: `ros2 run swarm_ros observer_cli`
- Status via: `ros2 topic echo /swarm/status`

**Phase 9.7b (Future):** Web dashboard
- Browser-based map view
- Point-and-click commands
- Real-time visualization

---

### 9.8 Tactical Mission Controller

**Purpose:** Main orchestrator integrating all components.

**File:** `swarm/tactical/mission_controller.py`

#### Main Loop (10 Hz)

```python
async def run(self):
    while self.state != TacticalState.RTL:
        # 1. Process observer commands (highest priority)
        await self._process_observer_commands()

        # 2. Update group position
        self.group_tracker.update()

        # 3. Check for threats
        threats = self.threat_detector.get_threats()

        # 4. State machine transitions
        await self._execute_state()

        # 5. Update GPS denial status
        self._update_jamming_status()

        # 6. Compute and send drone commands
        await self._send_drone_commands()

        await asyncio.sleep(0.1)
```

#### State Handlers

| State | Actions |
|-------|---------|
| INITIALIZING | Wait for all drones, initialize components |
| COVERING | Compute mesh, send position commands, monitor for threats |
| THREAT_DETECTED | Evaluate threat, compute intercept solutions, select defender |
| INTERCEPTING | Monitor intercept progress, maintain coverage with remaining drones |
| REGROUPING | Reform mesh after drone loss, redistribute positions |
| EMERGENCY | Handle critical failures, prepare for RTL |
| RTL | Return all drones to launch |

---

### 9.9 Test Scenarios

**File:** `scripts/test_phase9.py`

| Scenario | Description | Success Criteria |
|----------|-------------|------------------|
| `basic_coverage` | 6 drones, moving group, no threats | Mesh follows group, spacing maintained |
| `gps_jamming` | Group enters jamming zone | Jammed drones maintain relative position |
| `single_threat` | One enemy drone approaches | Detect → select → intercept → regroup |
| `multi_threat` | Two simultaneous threats | Sequential handling, correct prioritization |
| `threat_during_jamming` | Threat while partially jammed | GPS-denied drone selected for intercept |
| `observer_override` | Manual hostile marking | Immediate intercept on command |
| `full_mission` | Complete labyrinth traversal | Coverage maintained throughout |

---

### 9.10 Labyrinth World

**Files:**
- `worlds/labyrinth.sdf.jinja` - Jinja2 template
- `scripts/generate_labyrinth.py` - Generator script

**Deferred** - Will design after core components working.

#### Planned Features

- Parameterized maze generation (size, complexity)
- Configurable jamming zone placement
- Spawn points for group, drones, enemies
- Multiple terrain variants

---

## Reuse Matrix

| Existing Component | Location | Used By | Modifications |
|-------------------|----------|---------|---------------|
| `SwarmController` | `swarm/coordination/` | MissionController | Add tactical command methods |
| `FormationType` | `swarm/coordination/formations.py` | CoverageMesh | Reference only |
| `YOLODetector` | `swarm/perception/detector.py` | ThreatDetector | Add drone class |
| `SimpleTracker` | `swarm/perception/tracker.py` | ThreatDetector | None |
| `NeighborTracker` | `swarm_ros/.../neighbor_tracker.py` | ThreatDetector, RelativePos | None |
| `VIOEstimator` | `swarm/navigation/vio_estimator.py` | RelativePositioning | None |
| `NavigationEKF` | `swarm/navigation/ekf.py` | RelativePositioning | None |
| `GPSJammer` | `swarm/navigation/gps_jammer.py` | MissionController | None |
| `PursuitController` | `swarm/coordination/pursuit_controller.py` | InterceptPlanner | Reference for concepts |

---

## File Structure

```
swarm/
├── tactical/                          # NEW - Phase 9
│   ├── __init__.py
│   ├── config.py                     # Configuration dataclasses
│   ├── group_tracker.py              # 9.1
│   ├── coverage_mesh.py              # 9.2
│   ├── relative_positioning.py       # 9.3
│   ├── threat_detector.py            # 9.4
│   ├── intercept_planner.py          # 9.5
│   ├── defender_selector.py          # 9.6
│   ├── observer_interface.py         # 9.7
│   └── mission_controller.py         # 9.8
│
├── coordination/                      # EXISTING - Minor additions
│   └── swarm_controller.py           # Add tactical command methods
│
└── perception/                        # EXISTING - Minor additions
    └── detector.py                   # Verify drone class support

swarm_ros/swarm_ros/
└── observer_node.py                  # NEW - 9.7 ROS2 wrapper

scripts/
├── generate_drone_dataset.py         # NEW - 9.4b
├── observer_cli.py                   # NEW - 9.7
├── test_phase9.py                    # NEW - 9.9
└── generate_labyrinth.py             # NEW - 9.10 (deferred)

tests/
├── test_group_tracker.py             # NEW
├── test_coverage_mesh.py             # NEW
├── test_relative_positioning.py      # NEW
├── test_threat_detector.py           # NEW
├── test_intercept_planner.py         # NEW
├── test_defender_selector.py         # NEW
└── test_tactical_mission.py          # NEW

data/
└── drone_detection/                  # NEW - 9.4b
    ├── images/
    ├── labels/
    └── weights/

worlds/
└── labyrinth.sdf.jinja               # NEW - 9.10 (deferred)
```

---

## Implementation Order

```
Phase │ Component              │ Dependencies      │ Testable Alone?
──────┼────────────────────────┼───────────────────┼─────────────────
  1   │ 9.1 GroupTracker       │ Existing YOLO     │ Yes (mock GPS)
  2   │ 9.2 CoverageMesh       │ 9.1               │ Yes (static group)
  3   │ 9.3 RelativePositioning│ Existing VIO      │ Yes (mock jamming)
  4   │ 9.4b DroneDataset      │ Gazebo            │ Yes (training only)
  5   │ 9.4 ThreatDetector     │ 9.4b, YOLO        │ Yes (recorded video)
  6   │ 9.5 InterceptPlanner   │ None              │ Yes (unit tests)
  7   │ 9.6 DefenderSelector   │ 9.2, 9.5          │ Yes (mock scenarios)
  8   │ 9.7 ObserverInterface  │ None              │ Yes (CLI tests)
  9   │ 9.8 MissionController  │ All above         │ No (integration)
 10   │ 9.9 TestScenarios      │ 9.8               │ No (needs full system)
 11   │ 9.10 LabyrinthWorld    │ None              │ Yes (Gazebo only)
```

---

## Dependency Graph

```
                         ┌─────────────────┐
                         │  9.8 Mission    │
                         │   Controller    │
                         └────────┬────────┘
                                  │
         ┌────────────────────────┼────────────────────┐
         │                        │                    │
         ▼                        ▼                    ▼
┌───────────────┐        ┌───────────────┐    ┌───────────────┐
│ 9.7 Observer  │        │ 9.6 Defender  │    │ 9.2 Coverage  │
│   Interface   │        │   Selector    │    │     Mesh      │
└───────────────┘        └───────┬───────┘    └───────┬───────┘
                                 │                    │
                         ┌───────┴───────┐            │
                         ▼               ▼            ▼
                 ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
                 │9.5 Intercept│ │9.4 Threat   │ │9.1 Group    │
                 │   Planner   │ │  Detector   │ │  Tracker    │
                 └─────────────┘ └──────┬──────┘ └─────────────┘
                                        │
                                ┌───────┴───────┐
                                ▼               ▼
                         ┌─────────────┐ ┌─────────────┐
                         │9.4b Drone   │ │9.3 Relative │
                         │  Dataset    │ │ Positioning │
                         └─────────────┘ └──────┬──────┘
                                                │
                                                ▼
                                 ┌─────────────────────────┐
                                 │ EXISTING INFRASTRUCTURE │
                                 │ VIO, NeighborTracker,   │
                                 │ GPSJammer, YOLO,        │
                                 │ SwarmController         │
                                 └─────────────────────────┘
```

---

## Estimated Effort

| Component | New Lines | Complexity | Notes |
|-----------|-----------|------------|-------|
| 9.1 GroupTracker | ~250 | Medium | Fusion logic |
| 9.2 CoverageMesh | ~350 | Medium | Dynamic formation |
| 9.3 RelativePositioning | ~250 | Medium | VIO integration |
| 9.4 ThreatDetector | ~300 | Medium | Classification logic |
| 9.4b DroneDataset | ~200 | Low | Scripting |
| 9.5 InterceptPlanner | ~200 | Low | Pure math |
| 9.6 DefenderSelector | ~150 | Low | Scoring algorithm |
| 9.7 ObserverInterface | ~300 | Medium | API design |
| 9.8 MissionController | ~500 | High | Integration |
| 9.9 TestScenarios | ~400 | Medium | Test design |
| **Total** | **~2,900** | | |

---

## Decisions Log

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Group tracking | GPS beacons + visual fallback | Hybrid handles jamming zones |
| Mesh pattern | Hexagonal | Best coverage efficiency |
| Threat dataset | Synthetic from simulation | Controllable, sufficient for sim |
| Observer interface | CLI + ROS2 first | Fast to implement, extensible later |
| Intercept policy | Always intercept | User requirement, simplifies logic |
| Defender selection | Weighted multi-factor | Balances speed, probability, expendability |

---

## Open Questions

1. **Labyrinth dimensions?** - Deferred to later discussion
2. **Enemy drone behavior?** - Simple linear flight? Evasive maneuvers?
3. **Multiple observers?** - Conflict resolution if commands disagree?
4. **Drone recovery?** - Can interceptor survive collision? Or always sacrificed?
5. **Communication model?** - Perfect comms? Simulated packet loss?

---

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Coverage maintenance | >90% of group covered | Geometric calculation |
| Threat detection latency | <2 seconds | Time from appearance to classification |
| Intercept success rate | >80% | Collision achieved / attempts |
| Formation recovery time | <10 seconds | Time to reform after intercept |
| GPS-denied operation | Functional | Formation holds with 50% drones jammed |

---

## References

- Phase 6: GPS-Denied Navigation (VIO, EKF)
- Phase 7: Autonomous Missions (GPS jamming, pursuit)
- Existing: `swarm/coordination/pursuit_controller.py`
- Existing: `swarm/navigation/gps_jammer.py`
