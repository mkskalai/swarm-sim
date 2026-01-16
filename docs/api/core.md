# Core API Reference

The `swarm.core` module provides fundamental drone control and configuration.

## Drone

Single drone controller using MAVSDK.

**Location:** `swarm/core/drone.py`

```python
from swarm.core import Drone, DroneConfig

drone = Drone(DroneConfig(system_address="udp://:14540"))
await drone.connect()
await drone.arm()
await drone.takeoff(altitude=10.0)
await drone.goto_position_ned(north=50, east=0, down=-10, yaw=0)
await drone.land()
```

### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `connect(timeout)` | `timeout: float = 30.0` | `bool` | Connect to drone |
| `disconnect()` | - | `None` | Disconnect |
| `arm()` | - | `bool` | Arm motors |
| `disarm()` | - | `bool` | Disarm motors |
| `takeoff(altitude)` | `altitude: float = 10.0` | `bool` | Takeoff to altitude |
| `land()` | - | `bool` | Land at current position |
| `return_to_launch()` | - | `bool` | Return to home position |
| `goto_location(lat, lon, alt, yaw)` | GPS coords | `bool` | Fly to GPS location |
| `goto_position_ned(n, e, d, yaw, tolerance)` | NED coords, meters | `bool` | Fly to local position |
| `start_offboard()` | - | `bool` | Enter offboard control mode |
| `stop_offboard()` | - | `bool` | Exit offboard control mode |
| `set_position_ned(n, e, d, yaw)` | NED coords | `bool` | Set position setpoint |
| `set_velocity_ned(vn, ve, vd, yaw_rate)` | m/s, deg/s | `bool` | Set velocity setpoint |

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `position` | `Position` | Current NED position (north, east, down, yaw) |
| `global_position` | `GlobalPosition` | GPS position (lat, lon, alt) |
| `velocity` | `Velocity` | Current velocity (vn, ve, vd) |
| `battery_level` | `float` | Battery percentage (0-100) |
| `is_connected` | `bool` | Connection status |
| `is_armed` | `bool` | Arm status |
| `state` | `DroneState` | Current state enum |

### DroneState Enum

```python
class DroneState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    IN_FLIGHT = "in_flight"
    LANDING = "landing"
    LANDED = "landed"
```

---

## DroneConfig

Configuration dataclass for a single drone.

**Location:** `swarm/core/config.py`

```python
from swarm.core import DroneConfig

# Default config
config = DroneConfig()

# Custom config
config = DroneConfig(
    system_address="udp://:14540",
    instance_id=0,
    connection_timeout=30.0,
    default_altitude=10.0,
)

# Generate config for instance N
config = DroneConfig.for_instance(instance_id=2, base_port=14540)
# Results in system_address="udp://:14560"
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `system_address` | `str` | `"udp://:14540"` | MAVSDK connection string |
| `instance_id` | `int` | `0` | Drone instance number |
| `connection_timeout` | `float` | `30.0` | Connection timeout (seconds) |
| `action_timeout` | `float` | `10.0` | Action timeout (seconds) |
| `default_altitude` | `float` | `10.0` | Default takeoff altitude (meters) |
| `default_speed` | `float` | `5.0` | Default flight speed (m/s) |
| `position_update_rate` | `float` | `20.0` | Telemetry update rate (Hz) |

### Class Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `for_instance(instance_id, base_port)` | `int`, `int` | `DroneConfig` | Generate config for instance |

---

## Fleet

Multi-drone manager (MAVSDK-based, legacy).

**Location:** `swarm/core/fleet.py`

> **Note:** For multi-drone operations, prefer `SwarmController` from `swarm.coordination` which uses pymavlink and handles port isolation correctly.

```python
from swarm.core import Fleet, FleetConfig

fleet = Fleet(FleetConfig(num_drones=3))
await fleet.connect_all()
await fleet.arm_all()
await fleet.takeoff_all(altitude=10.0)
await fleet.land_all()
```

### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `connect_all(timeout)` | `timeout: float = 30.0` | `bool` | Connect all drones |
| `disconnect_all()` | - | `None` | Disconnect all |
| `arm_all()` | - | `bool` | Arm all drones |
| `disarm_all()` | - | `bool` | Disarm all drones |
| `takeoff_all(altitude)` | `altitude: float = 10.0` | `bool` | Takeoff all drones |
| `land_all()` | - | `bool` | Land all drones |
| `goto_positions(positions, tolerance)` | `list[Position]`, `float` | `bool` | Move to positions |
| `get_positions()` | - | `list[Position]` | Get current positions |
| `get_battery_levels()` | - | `list[float]` | Get battery levels |

### Container Protocol

```python
len(fleet)        # Number of drones
fleet[0]          # Access drone by index
for drone in fleet:
    print(drone.position)
```

---

## FleetConfig

Configuration for Fleet class.

**Location:** `swarm/core/config.py`

```python
from swarm.core import FleetConfig

config = FleetConfig(
    num_drones=6,
    base_mavsdk_port=14540,
    base_fdm_port=9002,
    port_offset=10,
)
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `num_drones` | `int` | `3` | Number of drones |
| `base_mavsdk_port` | `int` | `14540` | Base MAVSDK port |
| `base_fdm_port` | `int` | `9002` | Base FDM port |
| `port_offset` | `int` | `10` | Port increment per drone |
| `formation_spacing` | `float` | `5.0` | Default formation spacing |

### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `get_drone_config(instance_id)` | `int` | `DroneConfig` | Config for instance |
| `get_fdm_port(instance_id)` | `int` | `int` | FDM port for instance |
| `get_mavsdk_port(instance_id)` | `int` | `int` | MAVSDK port for instance |
| `get_spawn_position(instance_id)` | `int` | `tuple[float, float, float]` | Spawn position |
