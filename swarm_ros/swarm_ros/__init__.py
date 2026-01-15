"""
swarm_ros - ROS2 package for drone swarm P2P communication.

This package provides:
- DroneState/DroneIntent/FormationRole messages for P2P sharing
- Detection/DetectionArray messages for object detection sharing
- SwarmBridge: Bridges existing SwarmController to ROS2 (simulation)
- DroneNode: Per-drone controller with local pymavlink (hardware)
- MissionCoordinator: GCS-level mission planning and monitoring
- NeighborTracker: P2P neighbor discovery and collision avoidance
- DetectionTracker: P2P detection cache and aggregation
- PerceptionNode: YOLOv11-based object detection

Usage (simulation):
    ros2 launch swarm_ros simulation.launch.py num_drones:=3

Usage (perception):
    ros2 launch swarm_ros perception.launch.py num_drones:=3

Usage (hardware):
    # On GCS:
    ros2 run swarm_ros mission_coordinator --ros-args -p num_drones:=6

    # On each drone's onboard computer:
    ros2 run swarm_ros drone_node --ros-args -p drone_id:=0 -p num_drones:=6
"""

__version__ = "0.1.0"

from swarm_ros.neighbor_tracker import (
    NeighborTracker,
    NeighborTrackerConfig,
    NeighborState,
    CollisionWarning,
)

from swarm_ros.detection_tracker import (
    DetectionTracker,
    DetectionTrackerConfig,
    CachedDetection,
)

__all__ = [
    "NeighborTracker",
    "NeighborTrackerConfig",
    "NeighborState",
    "CollisionWarning",
    "DetectionTracker",
    "DetectionTrackerConfig",
    "CachedDetection",
]
