"""
MissionCoordinator - GCS-level mission planning and swarm coordination.

Runs on Ground Control Station to:
- Monitor all drone states via ROS2 subscriptions
- Provide services for formation changes and mission assignment
- Publish aggregated swarm status
- Coordinate distributed formation flying (assigns roles to DroneNodes)

Example:
    ros2 run swarm_ros mission_coordinator --ros-args -p num_drones:=6
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


@dataclass
class DroneStatus:
    """Tracked status of a single drone."""

    drone_id: int
    position_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0
    state: int = 0  # DroneState enum value
    battery_percent: int = 100
    is_healthy: bool = False
    last_update: float = 0.0

    @property
    def is_stale(self) -> bool:
        """Check if status is stale (no update for >3s)."""
        return time.time() - self.last_update > 3.0 if self.last_update > 0 else True


class MissionCoordinator(Node):
    """
    GCS-level mission coordinator for drone swarm.

    Publishers:
        /swarm/status (SwarmStatus) @ 1Hz - Aggregated swarm status

    Subscribers:
        /drone_N/state - All drone states

    Services:
        /swarm/set_formation (SetFormation) - Assign formation to swarm
        /swarm/emergency (EmergencyCommand) - Emergency commands
    """

    def __init__(self):
        super().__init__("mission_coordinator")

        # Declare parameters
        self.declare_parameter("num_drones", 3)

        self.num_drones = self.get_parameter("num_drones").value

        # Import swarm modules for formation calculations
        try:
            from swarm.coordination.formations import (
                FormationCalculator,
                FormationType,
                FormationConfig,
            )

            self._FormationCalculator = FormationCalculator
            self._FormationType = FormationType
            self._FormationConfig = FormationConfig
            self.formation_calc = FormationCalculator()
        except ImportError as e:
            self.get_logger().warning(f"swarm.coordination not available: {e}")
            self.formation_calc = None
            self._FormationType = None

        # Import ROS2 messages
        try:
            from swarm_ros.msg import DroneState, SwarmStatus, FormationRole
            from swarm_ros.srv import SetFormation, EmergencyCommand

            self._DroneState = DroneState
            self._SwarmStatus = SwarmStatus
            self._FormationRole = FormationRole
            self._SetFormation = SetFormation
            self._EmergencyCommand = EmergencyCommand
        except ImportError as e:
            self.get_logger().error(f"swarm_ros messages not built: {e}")
            raise

        # QoS profiles
        self._reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # State tracking
        self._drones: dict[int, DroneStatus] = {}
        for i in range(self.num_drones):
            self._drones[i] = DroneStatus(drone_id=i)

        self._lock = threading.Lock()

        # Current formation state
        self._current_formation_id = 0
        self._current_formation_type = 0
        self._mission_name = ""
        self._mission_progress = 0.0

        # Publishers
        self.status_pub = self.create_publisher(
            self._SwarmStatus, "/swarm/status", self._reliable_qos
        )

        # Formation role publishers (to each drone)
        self.role_pubs = {}
        for i in range(self.num_drones):
            pub = self.create_publisher(
                self._FormationRole,
                f"/drone_{i}/formation_command",
                self._reliable_qos,
            )
            self.role_pubs[i] = pub

        # Subscribers
        for i in range(self.num_drones):
            self.create_subscription(
                self._DroneState,
                f"/drone_{i}/state",
                lambda msg, did=i: self._on_drone_state(did, msg),
                self._best_effort_qos,
            )

        # Services
        self.create_service(
            self._SetFormation, "/swarm/set_formation", self._handle_set_formation
        )
        self.create_service(
            self._EmergencyCommand, "/swarm/emergency", self._handle_emergency
        )

        # Timers
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"MissionCoordinator initialized for {self.num_drones} drones"
        )

    def _on_drone_state(self, drone_id: int, msg) -> None:
        """Handle incoming drone state."""
        with self._lock:
            status = self._drones.get(drone_id)
            if status is None:
                status = DroneStatus(drone_id=drone_id)
                self._drones[drone_id] = status

            status.position_ned = tuple(msg.position_ned)
            status.velocity_ned = tuple(msg.velocity_ned)
            status.yaw_deg = msg.yaw_deg
            status.state = msg.state
            status.battery_percent = msg.battery_percent
            status.is_healthy = msg.is_healthy
            status.last_update = time.time()

    def _publish_status(self) -> None:
        """Publish aggregated swarm status."""
        msg = self._SwarmStatus()
        msg.num_drones_configured = self.num_drones

        with self._lock:
            # Count states
            connected = 0
            armed = 0
            in_flight = 0
            failed = 0

            drone_states = []
            for drone_id, status in self._drones.items():
                if status.is_stale:
                    failed += 1
                    continue

                connected += 1
                if status.state >= self._DroneState.STATE_ARMED:
                    armed += 1
                if status.state == self._DroneState.STATE_IN_FLIGHT:
                    in_flight += 1

                # Create DroneState message for array
                ds = self._DroneState()
                ds.drone_id = drone_id
                ds.position_ned = list(status.position_ned)
                ds.velocity_ned = list(status.velocity_ned)
                ds.yaw_deg = status.yaw_deg
                ds.state = status.state
                ds.battery_percent = status.battery_percent
                ds.is_healthy = status.is_healthy
                ds.stamp = self.get_clock().now().to_msg()
                drone_states.append(ds)

            msg.num_drones_connected = connected
            msg.num_drones_armed = armed
            msg.num_drones_in_flight = in_flight
            msg.num_drones_failed = failed
            msg.drone_states = drone_states

        # Determine swarm state
        if connected == 0:
            msg.swarm_state = msg.SWARM_IDLE
        elif armed == 0:
            msg.swarm_state = msg.SWARM_READY
        elif in_flight == 0:
            msg.swarm_state = msg.SWARM_ARMING
        elif self._current_formation_type > 0:
            msg.swarm_state = msg.SWARM_IN_FORMATION
        else:
            msg.swarm_state = msg.SWARM_EXECUTING_MISSION

        msg.current_formation_type = self._current_formation_type
        msg.current_formation_id = self._current_formation_id
        msg.current_mission_name = self._mission_name
        msg.mission_progress = self._mission_progress
        msg.stamp = self.get_clock().now().to_msg()

        self.status_pub.publish(msg)

    def _handle_set_formation(self, request, response):
        """Handle set_formation service - distribute roles to drones."""
        if self.formation_calc is None:
            response.success = False
            response.message = "Formation calculator not available"
            return response

        try:
            formation_type = self._FormationType(request.formation_type)
        except (ValueError, TypeError):
            response.success = False
            response.message = f"Invalid formation type: {request.formation_type}"
            return response

        # Get active drones
        with self._lock:
            active_drones = [
                did for did, status in self._drones.items() if not status.is_stale
            ]

        if len(active_drones) == 0:
            response.success = False
            response.message = "No active drones"
            return response

        # Calculate formation positions
        config = self._FormationConfig(
            spacing=request.spacing if request.spacing > 0 else 5.0,
            altitude=request.altitude if request.altitude > 0 else 10.0,
            heading=request.heading,
        )

        positions = self.formation_calc.calculate(
            formation_type, len(active_drones), config
        )

        # Increment formation ID
        self._current_formation_id += 1
        self._current_formation_type = request.formation_type

        # Assign roles - drone 0 is leader
        leader_id = active_drones[0]
        leader_pos = positions[0]

        self.get_logger().info(
            f"Assigning formation {formation_type.name} to {len(active_drones)} drones, "
            f"leader={leader_id}"
        )

        # Publish role assignments to each drone
        for i, drone_id in enumerate(active_drones):
            role_msg = self._FormationRole()
            role_msg.drone_id = drone_id
            role_msg.formation_id = self._current_formation_id
            role_msg.formation_type = request.formation_type
            role_msg.position_index = i

            if i == 0:
                # Leader
                role_msg.role = role_msg.ROLE_LEADER
                role_msg.leader_id = drone_id
                role_msg.offset_from_leader = [0.0, 0.0, 0.0]
            else:
                # Follower - calculate offset from leader
                role_msg.role = role_msg.ROLE_FOLLOWER
                role_msg.leader_id = leader_id
                offset = (
                    positions[i][0] - leader_pos[0],
                    positions[i][1] - leader_pos[1],
                    positions[i][2] - leader_pos[2],
                )
                role_msg.offset_from_leader = list(offset)

            role_msg.stamp = self.get_clock().now().to_msg()

            # Publish to drone's command topic
            if drone_id in self.role_pubs:
                self.role_pubs[drone_id].publish(role_msg)
                self.get_logger().info(
                    f"  Drone {drone_id}: "
                    f"{'LEADER' if i == 0 else 'FOLLOWER'}, "
                    f"offset={role_msg.offset_from_leader}"
                )

        response.success = True
        response.message = f"Formation {formation_type.name} assigned to {len(active_drones)} drones"
        response.drones_in_position = len(active_drones)
        response.arrival_time_seconds = 0.0

        return response

    def _handle_emergency(self, request, response):
        """Handle emergency command - broadcast to all drones."""
        # TODO: Call emergency service on each DroneNode
        # For now, just acknowledge
        response.success = True
        response.message = f"Emergency command {request.command} broadcast"
        response.drones_acknowledged = self.num_drones
        return response

    def get_swarm_positions(self) -> dict[int, tuple[float, float, float]]:
        """Get current positions of all active drones."""
        with self._lock:
            return {
                did: status.position_ned
                for did, status in self._drones.items()
                if not status.is_stale
            }

    def destroy_node(self):
        """Clean up on shutdown."""
        self.get_logger().info("Shutting down MissionCoordinator...")
        super().destroy_node()


def main(args=None):
    """Entry point for mission_coordinator."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    rclpy.init(args=args)

    try:
        node = MissionCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"MissionCoordinator failed: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
