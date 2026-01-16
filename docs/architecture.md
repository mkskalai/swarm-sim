# Architecture

This document describes the system architecture of the Drone Swarm Simulation Platform.

## Overview

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
                   │ pymavlink (port-isolated connections)
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
│  │  ardupilot_gazebo plugin ◄─► MAVLink ◄─► pymavlink        │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## Component Layers

### 1. Ground Control Station (GCS)

The centralized coordination layer responsible for:

- **Mission Planning**: Define waypoints, objectives, and constraints
- **Swarm Coordination**: Assign formations, distribute tasks
- **Monitoring**: Aggregate telemetry, track health status
- **High-level Decisions**: Abort missions, return-to-launch, replanning

### 2. Drone Swarm Layer

Individual drones with peer-to-peer capabilities:

- **Local Behaviors**: Obstacle avoidance, formation maintenance
- **Neighbor Communication**: Share position, intent, and detections via ROS2
- **Reactive Control**: Low-latency responses independent of GCS

### 3. Simulation Layer

- **Gazebo Harmonic**: Physics simulation, sensor emulation, camera rendering
- **ArduPilot SITL**: Flight controller simulation (one instance per drone)
- **ardupilot_gazebo Plugin**: JSON interface between Gazebo and SITL

---

## Key Architecture Decisions

### 1. pymavlink vs MAVSDK-Python vs MAVROS2

**Choice: pymavlink**

| Option | Pros | Cons |
|--------|------|------|
| **pymavlink** | Port isolation works, lightweight, full control | Manual message handling, synchronous |
| MAVSDK-Python | Clean async API, abstracts complexity | Routes by sysid - fails with same-sysid drones |
| MAVROS2 | Tight ROS2 integration | Heavy, complex setup, one node per drone |

**Why:** MAVSDK routes commands by MAVLink system ID. All SITL instances default to sysid=1, so MAVSDK sends all commands to the same drone regardless of port. pymavlink uses raw sockets - each connection is isolated by port.

For real hardware with onboard computers (1:1 connection per drone), pymavlink matches the architecture perfectly.

### 2. Hybrid Control Architecture

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

**Why:** Pure centralized fails if GCS connection drops. Pure decentralized is complex for coordinated missions. Hybrid gives the best of both - structured missions with resilient local behaviors.

### 3. Gazebo Harmonic + ardupilot_gazebo

- Harmonic is the LTS release matching ROS2 Jazzy
- ardupilot_gazebo is the official, maintained ArduPilot plugin
- Supports multiple SITL instances (critical for swarms)
- Modern gz-sim transport (not legacy gazebo-classic)

### 4. ROS2 Integration Strategy

**Simulation mode (SwarmBridge):**
- Wraps existing SwarmController
- Exposes coordination via ROS2 topics/services
- No changes to proven pymavlink code

**Hardware mode (DroneNode):**
- Runs on each drone's onboard computer
- Local pymavlink to flight controller
- P2P communication via ROS2 topics

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

---

## Port Scheme

Each drone requires unique ports for communication:

| Drone | Instance | FDM Port (Gazebo↔SITL) | MAVLink Port (pymavlink) |
|-------|----------|------------------------|--------------------------|
| Drone1 | 0 | 9002 | 14540 |
| Drone2 | 1 | 9012 | 14541 |
| Drone3 | 2 | 9022 | 14542 |
| DroneN | N-1 | 9002 + (N-1)*10 | 14540 + (N-1) |

**FDM Ports:** Communication between Gazebo's ArduPilotPlugin and SITL. Each drone model has a unique `fdm_port_in` in its SDF file.

**MAVLink Ports:** Communication between pymavlink and SITL. Configured via `--out=udp:127.0.0.1:PORT` when launching SITL with MAVProxy.

---

## Data Flow

### Telemetry Flow (Simulation)
```
Gazebo → ArduPilotPlugin → SITL → MAVProxy → pymavlink → SwarmController → ROS2 Topics
```

### Command Flow (Simulation)
```
ROS2 Service → SwarmBridge → SwarmController → pymavlink → MAVProxy → SITL → ArduPilotPlugin → Gazebo
```

### Camera/Perception Flow
```
Gazebo Camera → ros_gz_bridge → /drone_N/camera/image → PerceptionNode → /drone_N/detections
```

---

## Hardware Architecture (Future)

For real drones with onboard computers:

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
     Other Drones / GCS
```

Key differences from simulation:
- 1:1 pymavlink connection (no system ID routing issues)
- ROS2 P2P over WiFi mesh
- Local perception processing on each drone
