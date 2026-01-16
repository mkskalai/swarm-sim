# ROS2 API Reference

The `swarm_ros` package provides ROS2 integration for the drone swarm.

## Package Overview

```
swarm_ros/
├── msg/                    # Custom message definitions
├── srv/                    # Custom service definitions
├── swarm_ros/              # Python nodes
│   ├── swarm_bridge.py     # Simulation bridge
│   ├── drone_node.py       # Hardware node
│   ├── perception_node.py  # Detection node
│   └── neighbor_tracker.py # P2P tracking
└── launch/                 # Launch files
```

---

## Messages

### DroneState.msg

Per-drone telemetry, published at 10Hz.

```
uint8 drone_id
float32[3] position_ned     # North, East, Down (meters)
float32[3] velocity_ned     # m/s
float32 yaw_deg
uint8 state                 # See state constants below
uint8 battery_percent
bool is_healthy
bool has_gps_lock
bool ekf_ok
builtin_interfaces/Time stamp
```

**State Constants:**
| Value | State |
|-------|-------|
| 0 | DISCONNECTED |
| 1 | CONNECTED |
| 2 | ARMED |
| 3 | TAKING_OFF |
| 4 | IN_FLIGHT |
| 5 | LANDING |
| 6 | LANDED |

### DroneIntent.msg

Per-drone intentions, published at 4Hz.

```
uint8 drone_id
float32[3] target_position_ned
uint8 action_type           # See action constants below
uint16 action_id
float32 eta_seconds
```

**Action Constants:**
| Value | Action |
|-------|--------|
| 0 | HOVER |
| 1 | GOTO |
| 2 | ORBIT |
| 3 | TRACK |
| 5 | RTL |
| 6 | LAND |

### FormationRole.msg

Formation membership, published at 4Hz.

```
uint8 drone_id
uint16 formation_id
uint8 role                  # 0=UNASSIGNED, 1=LEADER, 2=FOLLOWER
uint8 leader_id
float32[3] offset_from_leader
uint8 formation_type        # See formation constants
```

**Formation Constants:**
| Value | Formation |
|-------|-----------|
| 0 | LINE |
| 1 | V_FORMATION |
| 2 | TRIANGLE |
| 3 | GRID |
| 4 | CIRCLE |
| 5 | DIAMOND |

### SwarmStatus.msg

Aggregated swarm status, published at 1Hz.

```
uint8 num_drones
uint8 num_connected
uint8 num_armed
uint8 num_in_flight
uint8 swarm_state           # Overall state
uint8 active_formation
DroneState[] drone_states   # All drone states
builtin_interfaces/Time stamp
```

### Detection.msg

Single object detection.

```
uint8 drone_id
uint32 track_id             # 0 = untracked
uint8 class_id              # COCO class
string class_name
float32 confidence
uint16 bbox_x
uint16 bbox_y
uint16 bbox_width
uint16 bbox_height
float32[3] estimated_position_ned  # NaN if unknown
builtin_interfaces/Time stamp
```

### DetectionArray.msg

Array of detections from one drone.

```
uint8 drone_id
uint32 frame_seq
float32[3] drone_position_ned
float32 inference_time_ms
Detection[] detections
builtin_interfaces/Time stamp
```

---

## Services

### SetFormation.srv

Request a formation change.

**Request:**
```
uint8 formation_type    # 0-5, see formation constants
float32 spacing         # Distance between drones (meters)
float32 altitude        # Formation altitude (meters)
float32 duration        # Hold duration (seconds)
```

**Response:**
```
bool success
string message
```

**Example:**
```bash
ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation \
  "{formation_type: 1, spacing: 5.0, altitude: 10.0, duration: 30.0}"
```

### EmergencyCommand.srv

Emergency command for swarm or individual drone.

**Request:**
```
uint8 command           # 0=STOP, 1=RTL, 2=LAND, 3=HOVER
uint8 drone_id          # 255 = all drones
```

**Response:**
```
bool success
string message
```

---

## Topics

### Published by SwarmBridge

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/swarm/status` | SwarmStatus | 1Hz | Aggregated swarm status |
| `/drone_N/state` | DroneState | 10Hz | Per-drone telemetry |
| `/drone_N/intent` | DroneIntent | 4Hz | Per-drone intentions |
| `/drone_N/formation_role` | FormationRole | 4Hz | Formation membership |

### Published by PerceptionNode

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/drone_N/detections` | DetectionArray | ~15Hz | Object detections |

### Subscribed by PerceptionNode

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_N/camera/image` | sensor_msgs/Image | Camera feed from Gazebo |
| `/drone_N/state` | DroneState | Drone position for context |

---

## Service Endpoints

### Swarm Services (SwarmBridge)

| Service | Type | Description |
|---------|------|-------------|
| `/swarm/connect` | std_srvs/Trigger | Connect to all drones |
| `/swarm/arm_all` | std_srvs/Trigger | Arm all drones |
| `/swarm/takeoff_all` | std_srvs/Trigger | Takeoff all drones |
| `/swarm/land_all` | std_srvs/Trigger | Land all drones |
| `/swarm/set_formation` | SetFormation | Set formation |
| `/swarm/emergency` | EmergencyCommand | Emergency command |

### Per-Drone Services (DroneNode)

| Service | Type | Description |
|---------|------|-------------|
| `/drone_N/emergency` | EmergencyCommand | Per-drone emergency |

---

## Launch Files

### simulation.launch.py

Launch SwarmBridge for simulation mode.

```bash
ros2 launch swarm_ros simulation.launch.py num_drones:=3
ros2 launch swarm_ros simulation.launch.py num_drones:=3 enable_cameras:=true
```

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `num_drones` | 3 | Number of drones |
| `enable_cameras` | false | Bridge camera topics |

### perception.launch.py

Launch perception nodes for all drones.

```bash
ros2 launch swarm_ros perception.launch.py num_drones:=3
ros2 launch swarm_ros perception.launch.py num_drones:=3 use_gpu:=true
```

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `num_drones` | 3 | Number of drones |
| `use_gpu` | true | Use GPU for inference |
| `model_name` | yolo11n.pt | YOLO model to use |

### drone.launch.py

Launch DroneNode for hardware mode.

```bash
ros2 launch swarm_ros drone.launch.py drone_id:=0 num_drones:=6 serial_port:=/dev/ttyACM0
ros2 launch swarm_ros drone.launch.py drone_id:=0 num_drones:=3 use_udp:=true udp_port:=14540
```

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `drone_id` | required | This drone's ID (0-indexed) |
| `num_drones` | required | Total drones in swarm |
| `serial_port` | /dev/ttyACM0 | Serial port for flight controller |
| `use_udp` | false | Use UDP instead of serial |
| `udp_port` | 14540 | UDP port (if use_udp=true) |

---

## NeighborTracker

P2P neighbor tracking with collision avoidance.

**Location:** `swarm_ros/swarm_ros/neighbor_tracker.py`

```python
from swarm_ros import NeighborTracker, NeighborTrackerConfig

config = NeighborTrackerConfig(
    stale_timeout=3.0,
    failed_timeout=10.0,
    collision_horizon=5.0,
    safe_distance=3.0,
)

tracker = NeighborTracker(
    node=self,
    own_drone_id=0,
    num_drones=6,
    config=config,
)

# Register callbacks
tracker.on_neighbor_joined(lambda did: print(f"Neighbor {did} joined"))
tracker.on_neighbor_lost(lambda did: print(f"Neighbor {did} lost"))
tracker.on_collision_warning(lambda w: handle_collision(w))

tracker.start()

# In control loop
collision = tracker.predict_collision(own_position, own_velocity)
if collision and collision.time_to_collision < 2.0:
    # Execute avoidance using collision.avoidance_vector
    pass
```

### NeighborTrackerConfig

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `stale_timeout` | `float` | `3.0` | Seconds before marking stale |
| `failed_timeout` | `float` | `10.0` | Seconds before marking failed |
| `collision_horizon` | `float` | `5.0` | Lookahead time (seconds) |
| `safe_distance` | `float` | `3.0` | Minimum safe distance (meters) |

### CollisionWarning

Returned by `predict_collision()`.

| Field | Type | Description |
|-------|------|-------------|
| `other_drone_id` | `int` | ID of threatening drone |
| `time_to_collision` | `float` | Seconds until collision |
| `closest_approach` | `float` | Minimum distance (meters) |
| `avoidance_vector` | `tuple` | Suggested avoidance direction |

---

## Usage Examples

### Monitor Swarm Status

```python
import rclpy
from rclpy.node import Node
from swarm_ros.msg import SwarmStatus

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        self.sub = self.create_subscription(
            SwarmStatus, '/swarm/status', self.status_callback, 10)

    def status_callback(self, msg):
        print(f"Drones: {msg.num_connected}/{msg.num_drones} connected")
        print(f"In flight: {msg.num_in_flight}")
        for drone in msg.drone_states:
            print(f"  Drone {drone.drone_id}: {drone.position_ned}")

rclpy.init()
node = StatusMonitor()
rclpy.spin(node)
```

### Command Formation via Code

```python
import rclpy
from rclpy.node import Node
from swarm_ros.srv import SetFormation

class FormationCommander(Node):
    def __init__(self):
        super().__init__('formation_commander')
        self.client = self.create_client(SetFormation, '/swarm/set_formation')
        self.client.wait_for_service()

    def fly_v_formation(self):
        request = SetFormation.Request()
        request.formation_type = 1  # V_FORMATION
        request.spacing = 5.0
        request.altitude = 10.0
        request.duration = 30.0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

rclpy.init()
commander = FormationCommander()
result = commander.fly_v_formation()
print(f"Formation result: {result.success} - {result.message}")
```
