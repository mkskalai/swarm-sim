# Coordination API Reference

The `swarm.coordination` module provides swarm coordination, formations, and missions.

## SwarmController

Main orchestrator for multi-drone operations using pymavlink.

**Location:** `swarm/coordination/swarm_controller.py`

```python
from swarm.coordination import SwarmController, SwarmConfig, FormationType

controller = SwarmController(SwarmConfig(num_drones=3))
controller.connect_all()
controller.wait_for_ekf_all()
controller.arm_all()
controller.takeoff_all(altitude=10.0)
controller.fly_formation(FormationType.V_FORMATION, duration=15.0)
controller.land_all()
```

### Methods

#### Connection

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `connect_all(timeout)` | `timeout: float = 30.0` | `bool` | Connect to all drones |
| `disconnect_all()` | - | `None` | Disconnect all |

#### Flight Control

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `wait_for_ekf_all(timeout)` | `timeout: float = 60.0` | `bool` | Wait for EKF convergence |
| `set_mode_all(mode)` | `mode: str` | `bool` | Set flight mode |
| `arm_all()` | - | `bool` | Arm all drones |
| `disarm_all()` | - | `None` | Disarm all drones |
| `takeoff_all(altitude, wait)` | `float`, `bool` | `bool` | Takeoff all drones |
| `land_all()` | - | `None` | Land all drones |
| `return_to_launch_all()` | - | `None` | RTL all drones |

#### Formations

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `fly_formation(formation_type, duration, config, arrival_timeout, arrival_tolerance)` | See below | `bool` | Fly formation |
| `fly_formation_transition(from_type, to_type, transition_duration, hold_duration)` | See below | `bool` | Transition between formations |

**fly_formation parameters:**
- `formation_type: FormationType` - Formation pattern
- `duration: float = 15.0` - Hold duration after arrival
- `config: FormationConfig = None` - Formation parameters
- `arrival_timeout: float = 30.0` - Max time to reach positions
- `arrival_tolerance: float = 2.0` - Position tolerance (meters)

#### Leader-Follower

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `start_leader_follower(leader_id, formation_type)` | `int`, `FormationType` | `None` | Start leader-follower mode |
| `update_leader_follower(leader_position, duration, wait_for_arrival)` | `PositionNED`, `float`, `bool` | `None` | Move leader |

#### Missions

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `execute_mission(mission)` | `SwarmMission` | `bool` | Execute waypoint mission |

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `positions` | `dict[int, tuple]` | Current positions by drone ID |
| `num_drones` | `int` | Number of drones |
| `is_connected` | `bool` | All drones connected |

---

## SwarmConfig

Configuration for SwarmController.

**Location:** `swarm/coordination/swarm_controller.py`

```python
from swarm.coordination import SwarmConfig

config = SwarmConfig(
    num_drones=6,
    base_port=14540,
    heartbeat_timeout=3.0,
    connection_timeout=30.0,
    ekf_timeout=60.0,
)
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `num_drones` | `int` | `3` | Number of drones |
| `base_port` | `int` | `14540` | Base MAVLink port |
| `heartbeat_timeout` | `float` | `3.0` | Heartbeat timeout (seconds) |
| `position_update_rate` | `float` | `4.0` | Position polling rate (Hz) |
| `connection_timeout` | `float` | `30.0` | Connection timeout |
| `ekf_timeout` | `float` | `60.0` | EKF convergence timeout |

---

## FormationType

Enum of available formation patterns.

**Location:** `swarm/coordination/formations.py`

```python
from swarm.coordination import FormationType

formations = [
    FormationType.LINE,
    FormationType.V_FORMATION,
    FormationType.TRIANGLE,
    FormationType.GRID,
    FormationType.CIRCLE,
    FormationType.DIAMOND,
]
```

### Values

| Value | Integer | Description |
|-------|---------|-------------|
| `LINE` | 0 | Straight line |
| `V_FORMATION` | 1 | V-shape (geese pattern) |
| `TRIANGLE` | 2 | Equilateral triangle |
| `GRID` | 3 | Square/rectangular grid |
| `CIRCLE` | 4 | Circular arrangement |
| `DIAMOND` | 5 | Diamond pattern |

---

## FormationConfig

Parameters for formation geometry.

**Location:** `swarm/coordination/formations.py`

```python
from swarm.coordination import FormationConfig

config = FormationConfig(
    spacing=5.0,
    altitude=10.0,
    heading=0.0,
    center_north=0.0,
    center_east=0.0,
)
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `spacing` | `float` | `5.0` | Distance between drones (meters) |
| `altitude` | `float` | `10.0` | Formation altitude (meters, positive up) |
| `heading` | `float` | `0.0` | Formation heading (degrees) |
| `center_north` | `float` | `0.0` | Formation center north (meters) |
| `center_east` | `float` | `0.0` | Formation center east (meters) |

---

## Formation Functions

Low-level formation calculation functions.

**Location:** `swarm/coordination/formations.py`

```python
from swarm.coordination.formations import calculate_formation_positions

positions = calculate_formation_positions(
    formation_type=FormationType.V_FORMATION,
    num_drones=5,
    config=FormationConfig(spacing=5.0, altitude=10.0),
)
# Returns: [(n1, e1, d1), (n2, e2, d2), ...]
```

### Functions

| Function | Parameters | Returns | Description |
|----------|------------|---------|-------------|
| `calculate_formation_positions(formation_type, num_drones, config)` | `FormationType`, `int`, `FormationConfig` | `list[tuple]` | Calculate NED positions |
| `line_formation(num_drones, config)` | `int`, `FormationConfig` | `list[tuple]` | Line positions |
| `v_formation(num_drones, config)` | `int`, `FormationConfig` | `list[tuple]` | V positions |
| `triangle_formation(num_drones, config)` | `int`, `FormationConfig` | `list[tuple]` | Triangle positions |
| `grid_formation(num_drones, config)` | `int`, `FormationConfig` | `list[tuple]` | Grid positions |
| `circle_formation(num_drones, config)` | `int`, `FormationConfig` | `list[tuple]` | Circle positions |
| `diamond_formation(num_drones, config)` | `int`, `FormationConfig` | `list[tuple]` | Diamond positions |

---

## SwarmMission

Waypoint mission definition.

**Location:** `swarm/coordination/missions.py`

```python
from swarm.coordination import SwarmMission, Waypoint, FormationType

mission = SwarmMission(
    waypoints=[
        Waypoint(north=0, east=0, down=-10, hold_time=5.0),
        Waypoint(north=50, east=0, down=-10, hold_time=5.0),
        Waypoint(north=50, east=50, down=-10, hold_time=5.0),
    ],
    formation=FormationType.V_FORMATION,
    formation_spacing=5.0,
)
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `waypoints` | `list[Waypoint]` | required | Waypoints to visit |
| `formation` | `FormationType` | `LINE` | Formation during mission |
| `formation_spacing` | `float` | `5.0` | Formation spacing |

---

## Waypoint

Single waypoint definition.

**Location:** `swarm/coordination/missions.py`

```python
from swarm.coordination import Waypoint

wp = Waypoint(
    north=100.0,
    east=50.0,
    down=-15.0,  # Negative = above ground
    hold_time=10.0,
    speed=5.0,
)
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `north` | `float` | required | North position (meters) |
| `east` | `float` | required | East position (meters) |
| `down` | `float` | required | Down position (meters, negative = up) |
| `hold_time` | `float` | `0.0` | Time to hold at waypoint |
| `speed` | `float` | `5.0` | Approach speed (m/s) |

---

## PositionNED

Simple position container.

**Location:** `swarm/coordination/swarm_controller.py`

```python
from swarm.coordination import PositionNED

pos = PositionNED(north=10.0, east=20.0, down=-15.0)
```

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `north` | `float` | North position (meters) |
| `east` | `float` | East position (meters) |
| `down` | `float` | Down position (meters, negative = above ground) |
