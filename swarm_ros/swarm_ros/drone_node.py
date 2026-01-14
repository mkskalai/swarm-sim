"""
DroneNode - Per-drone ROS2 node for hardware deployment.

Each physical drone runs one DroneNode that:
- Connects locally to its flight controller via serial/USB (pymavlink)
- Publishes its state to P2P topics for neighbor awareness
- Subscribes to peer states for collision avoidance
- Receives high-level commands from GCS (MissionCoordinator)
- Executes formations based on its role (leader/follower)

Example (on onboard computer):
    ros2 run swarm_ros drone_node --ros-args \\
        -p drone_id:=0 \\
        -p num_drones:=6 \\
        -p serial_port:=/dev/ttyACM0
"""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class DroneAction(IntEnum):
    """Current action being executed by the drone."""

    IDLE = 0
    HOVER = 1
    GOTO = 2
    FORMATION = 3
    TRACKING = 4
    RTL = 5
    LANDING = 6


@dataclass
class TargetPosition:
    """Target position for drone movement."""

    north: float
    east: float
    down: float  # Negative = altitude
    yaw_deg: float = 0.0


class DroneNode(Node):
    """
    ROS2 node for individual drone control on real hardware.

    Publishers:
        /drone_{id}/state (DroneState) @ 10Hz - Own state for P2P sharing
        /drone_{id}/intent (DroneIntent) @ 4Hz - What drone intends to do
        /drone_{id}/formation_role (FormationRole) @ 4Hz - Formation membership

    Subscribers:
        /drone_*/state - All peer states via NeighborTracker
        /drone_{id}/command - Commands from GCS

    The node runs a 10Hz control loop that:
    1. Reads telemetry from flight controller
    2. Checks neighbors for collision avoidance
    3. Executes current action (formation, goto, etc.)
    4. Sends commands to flight controller
    5. Publishes own state
    """

    def __init__(self):
        super().__init__("drone_node")

        # Declare parameters
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("use_udp", False)  # For SITL testing
        self.declare_parameter("udp_port", 14540)

        self.drone_id = self.get_parameter("drone_id").value
        self.num_drones = self.get_parameter("num_drones").value
        self.serial_port = self.get_parameter("serial_port").value
        self.serial_baud = self.get_parameter("serial_baud").value
        self.use_udp = self.get_parameter("use_udp").value
        self.udp_port = self.get_parameter("udp_port").value

        # Import pymavlink
        try:
            from pymavlink import mavutil

            self._mavutil = mavutil
        except ImportError as e:
            self.get_logger().error(f"pymavlink not installed: {e}")
            raise

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
            self.get_logger().warning(
                f"swarm.coordination not available: {e}. "
                "Formation calculations will be limited."
            )
            self.formation_calc = None

        # Import ROS2 messages
        try:
            from swarm_ros.msg import DroneState, DroneIntent, FormationRole
            from swarm_ros.srv import EmergencyCommand

            self._DroneState = DroneState
            self._DroneIntent = DroneIntent
            self._FormationRole = FormationRole
            self._EmergencyCommand = EmergencyCommand
        except ImportError as e:
            self.get_logger().error(f"swarm_ros messages not built: {e}")
            raise

        # Import NeighborTracker
        from swarm_ros.neighbor_tracker import (
            NeighborTracker,
            NeighborTrackerConfig,
            CollisionWarning,
        )

        # QoS profiles
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers (P2P)
        self.state_pub = self.create_publisher(
            self._DroneState,
            f"/drone_{self.drone_id}/state",
            self._best_effort_qos,
        )
        self.intent_pub = self.create_publisher(
            self._DroneIntent,
            f"/drone_{self.drone_id}/intent",
            self._best_effort_qos,
        )
        self.role_pub = self.create_publisher(
            self._FormationRole,
            f"/drone_{self.drone_id}/formation_role",
            self._best_effort_qos,
        )

        # Services
        self.create_service(
            self._EmergencyCommand,
            f"/drone_{self.drone_id}/emergency",
            self._handle_emergency,
        )

        # Initialize NeighborTracker
        tracker_config = NeighborTrackerConfig(
            stale_timeout=3.0,
            failed_timeout=10.0,
            collision_horizon=5.0,
            safe_distance=3.0,
        )
        self.neighbor_tracker = NeighborTracker(
            self,
            own_drone_id=self.drone_id,
            num_drones=self.num_drones,
            config=tracker_config,
        )
        self.neighbor_tracker.on_neighbor_joined(self._on_neighbor_joined)
        self.neighbor_tracker.on_neighbor_lost(self._on_neighbor_lost)
        self.neighbor_tracker.on_collision_warning(self._on_collision_warning)

        # MAVLink connection (created in _connect())
        self.mavlink = None
        self._connected = False
        self._armed = False

        # Telemetry state
        self._position = (0.0, 0.0, 0.0)  # NED
        self._velocity = (0.0, 0.0, 0.0)  # NED
        self._yaw_deg = 0.0
        self._battery_percent = 100
        self._ekf_ok = False
        self._gps_fix = False

        # Current action
        self._action = DroneAction.IDLE
        self._action_id = 0
        self._target: TargetPosition | None = None

        # Formation state
        self._formation_id = 0
        self._formation_type = 0
        self._formation_role = 0  # ROLE_UNASSIGNED
        self._leader_id = 0
        self._formation_offset = (0.0, 0.0, 0.0)
        self._position_index = 0

        # Lock for thread safety
        self._lock = threading.Lock()

        # Timers
        self.create_timer(0.1, self._control_loop)  # 10 Hz main loop
        self.create_timer(0.25, self._publish_intent)  # 4 Hz intent
        self.create_timer(0.25, self._publish_role)  # 4 Hz role
        self.create_timer(1.0, self._check_neighbors)  # 1 Hz neighbor check

        self.get_logger().info(
            f"DroneNode {self.drone_id} initialized "
            f"(num_drones={self.num_drones})"
        )

        # Connect to flight controller
        self._connect()

        # Start neighbor tracking
        self.neighbor_tracker.start()

    def _connect(self) -> bool:
        """Connect to flight controller."""
        try:
            if self.use_udp:
                # UDP for SITL testing
                conn_str = f"udpin:0.0.0.0:{self.udp_port}"
            else:
                # Serial for real hardware
                conn_str = f"{self.serial_port}"

            self.get_logger().info(f"Connecting to {conn_str}...")

            if self.use_udp:
                self.mavlink = self._mavutil.mavlink_connection(conn_str)
            else:
                self.mavlink = self._mavutil.mavlink_connection(
                    conn_str, baud=self.serial_baud
                )

            # Wait for heartbeat
            self.mavlink.wait_heartbeat(timeout=30)
            self._connected = True
            self.get_logger().info(
                f"Connected! System {self.mavlink.target_system}, "
                f"Component {self.mavlink.target_component}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
            self._connected = False
            return False

    def _control_loop(self) -> None:
        """Main 10Hz control loop."""
        if not self._connected:
            return

        with self._lock:
            # 1. Read telemetry
            self._read_telemetry()

            # 2. Publish own state
            self._publish_state()

            # 3. Check for collision avoidance
            collision = self.neighbor_tracker.predict_collision(
                self._position, self._velocity
            )
            if collision and collision.time_to_collision < 2.0:
                self._handle_collision_avoidance(collision)
                return

            # 4. Execute current action
            if self._action == DroneAction.GOTO and self._target:
                self._execute_goto()
            elif self._action == DroneAction.FORMATION:
                self._execute_formation()

    def _read_telemetry(self) -> None:
        """Read telemetry from flight controller."""
        if not self.mavlink:
            return

        # Read multiple messages (non-blocking)
        for _ in range(10):
            msg = self.mavlink.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == "LOCAL_POSITION_NED":
                self._position = (msg.x, msg.y, msg.z)
                self._velocity = (msg.vx, msg.vy, msg.vz)

            elif msg_type == "ATTITUDE":
                self._yaw_deg = math.degrees(msg.yaw)

            elif msg_type == "SYS_STATUS":
                if msg.battery_remaining >= 0:
                    self._battery_percent = msg.battery_remaining

            elif msg_type == "EKF_STATUS_REPORT":
                # Check EKF flags (bits 3 and 4 for position)
                self._ekf_ok = (msg.flags & 0x18) != 0

            elif msg_type == "GPS_RAW_INT":
                self._gps_fix = msg.fix_type >= 3

    def _publish_state(self) -> None:
        """Publish own state for P2P sharing."""
        msg = self._DroneState()
        msg.drone_id = self.drone_id
        msg.position_ned = list(self._position)
        msg.velocity_ned = list(self._velocity)
        msg.yaw_deg = self._yaw_deg
        msg.battery_percent = self._battery_percent
        msg.is_healthy = self._connected and self._ekf_ok
        msg.has_gps_lock = self._gps_fix
        msg.ekf_ok = self._ekf_ok

        # Determine state
        if not self._connected:
            msg.state = msg.STATE_DISCONNECTED
        elif not self._armed:
            msg.state = msg.STATE_CONNECTED
        elif self._action == DroneAction.LANDING:
            msg.state = msg.STATE_LANDING
        elif self._action != DroneAction.IDLE:
            msg.state = msg.STATE_IN_FLIGHT
        else:
            msg.state = msg.STATE_ARMED

        msg.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

    def _publish_intent(self) -> None:
        """Publish current intent."""
        msg = self._DroneIntent()
        msg.drone_id = self.drone_id
        msg.action_id = self._action_id

        if self._target:
            msg.target_position_ned = [
                self._target.north,
                self._target.east,
                self._target.down,
            ]
            msg.target_yaw_deg = self._target.yaw_deg

            # Calculate ETA based on distance and typical speed
            dist = math.sqrt(
                (self._target.north - self._position[0]) ** 2
                + (self._target.east - self._position[1]) ** 2
                + (self._target.down - self._position[2]) ** 2
            )
            msg.eta_seconds = dist / 5.0  # Assume 5 m/s
        else:
            msg.target_position_ned = list(self._position)
            msg.target_yaw_deg = self._yaw_deg
            msg.eta_seconds = 0.0

        # Map action type
        action_map = {
            DroneAction.IDLE: msg.ACTION_HOVER,
            DroneAction.HOVER: msg.ACTION_HOVER,
            DroneAction.GOTO: msg.ACTION_GOTO,
            DroneAction.FORMATION: msg.ACTION_GOTO,
            DroneAction.TRACKING: msg.ACTION_TRACK,
            DroneAction.RTL: msg.ACTION_RTL,
            DroneAction.LANDING: msg.ACTION_LAND,
        }
        msg.action_type = action_map.get(self._action, msg.ACTION_HOVER)

        msg.stamp = self.get_clock().now().to_msg()
        self.intent_pub.publish(msg)

    def _publish_role(self) -> None:
        """Publish formation role."""
        msg = self._FormationRole()
        msg.drone_id = self.drone_id
        msg.formation_id = self._formation_id
        msg.role = self._formation_role
        msg.leader_id = self._leader_id
        msg.offset_from_leader = list(self._formation_offset)
        msg.formation_type = self._formation_type
        msg.position_index = self._position_index
        msg.stamp = self.get_clock().now().to_msg()
        self.role_pub.publish(msg)

    def _check_neighbors(self) -> None:
        """Periodic neighbor timeout check."""
        lost = self.neighbor_tracker.check_timeouts()
        if lost:
            self.get_logger().warning(f"Lost neighbors: {lost}")

    def _execute_goto(self) -> None:
        """Execute goto action - fly to target position."""
        if not self._target or not self.mavlink:
            return

        # Send position setpoint
        self.mavlink.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.mavlink.target_system,
            self.mavlink.target_component,
            self._mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask: position only
            self._target.north,
            self._target.east,
            self._target.down,
            0,
            0,
            0,  # velocity
            0,
            0,
            0,  # acceleration
            0,
            0,  # yaw, yaw_rate
        )

    def _execute_formation(self) -> None:
        """Execute formation - maintain position relative to leader."""
        if self._formation_role == 1:  # Leader
            # Leader just holds position
            return

        # Follower: calculate position based on leader
        leader_state = self.neighbor_tracker.get_neighbor(self._leader_id)
        if leader_state is None or leader_state.is_stale:
            self.get_logger().warning(
                f"Leader {self._leader_id} not available, holding position"
            )
            return

        # Target = leader position + offset
        target_n = leader_state.position_ned[0] + self._formation_offset[0]
        target_e = leader_state.position_ned[1] + self._formation_offset[1]
        target_d = leader_state.position_ned[2] + self._formation_offset[2]

        self._target = TargetPosition(
            north=target_n, east=target_e, down=target_d, yaw_deg=leader_state.yaw_deg
        )

        self._execute_goto()

    def _handle_collision_avoidance(self, collision) -> None:
        """React to imminent collision."""
        self.get_logger().warning(
            f"Collision warning! Drone {collision.other_drone_id} "
            f"in {collision.time_to_collision:.1f}s at {collision.min_distance:.1f}m"
        )

        # Send avoidance velocity
        avoidance = collision.avoidance_vector
        speed = 2.0  # m/s avoidance speed

        if self.mavlink:
            self.mavlink.mav.set_position_target_local_ned_send(
                0,
                self.mavlink.target_system,
                self.mavlink.target_component,
                self._mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # type_mask: velocity only
                0,
                0,
                0,  # position
                avoidance[0] * speed,
                avoidance[1] * speed,
                avoidance[2] * speed,
                0,
                0,
                0,  # acceleration
                0,
                0,  # yaw
            )

    # ----- Callbacks -----

    def _on_neighbor_joined(self, drone_id: int) -> None:
        """Callback when new neighbor discovered."""
        self.get_logger().info(f"Neighbor {drone_id} joined")

    def _on_neighbor_lost(self, drone_id: int) -> None:
        """Callback when neighbor lost."""
        self.get_logger().warning(f"Neighbor {drone_id} lost")

        # If we lost our leader, we may need to elect new leader
        if drone_id == self._leader_id and self._formation_role == 2:
            self.get_logger().warning("Leader lost! Need re-election")
            # TODO: Implement distributed leader election

    def _on_collision_warning(self, warning) -> None:
        """Callback for collision warning (in addition to control loop check)."""
        self.get_logger().warning(
            f"Collision warning with drone {warning.other_drone_id}"
        )

    # ----- Service Handlers -----

    def _handle_emergency(self, request, response):
        """Handle emergency command."""
        cmd = request.command

        with self._lock:
            if cmd == request.CMD_RTL:
                self._set_mode("RTL")
                self._action = DroneAction.RTL
                response.message = "RTL"
            elif cmd == request.CMD_LAND:
                self._set_mode("LAND")
                self._action = DroneAction.LANDING
                response.message = "Landing"
            elif cmd == request.CMD_HOVER:
                self._action = DroneAction.HOVER
                self._target = TargetPosition(
                    north=self._position[0],
                    east=self._position[1],
                    down=self._position[2],
                )
                response.message = "Hovering"
            elif cmd == request.CMD_DISARM:
                self._disarm()
                response.message = "Disarmed"
            else:
                response.success = False
                response.message = f"Unknown command: {cmd}"
                response.drones_acknowledged = 0
                return response

            response.success = True
            response.drones_acknowledged = 1

        return response

    # ----- MAVLink helpers -----

    def _set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.mavlink:
            return False

        mode_map = {
            "GUIDED": 4,
            "LAND": 9,
            "RTL": 6,
            "LOITER": 5,
            "STABILIZE": 0,
        }

        mode_id = mode_map.get(mode.upper())
        if mode_id is None:
            self.get_logger().error(f"Unknown mode: {mode}")
            return False

        self.mavlink.mav.set_mode_send(
            self.mavlink.target_system,
            self._mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        return True

    def _arm(self) -> bool:
        """Arm the drone."""
        if not self.mavlink:
            return False

        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self._armed = True
        return True

    def _disarm(self) -> bool:
        """Disarm the drone."""
        if not self.mavlink:
            return False

        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self._armed = False
        return True

    # ----- Public methods for external control -----

    def set_formation(
        self,
        formation_id: int,
        formation_type: int,
        role: int,
        leader_id: int,
        offset: tuple[float, float, float],
        position_index: int,
    ) -> None:
        """Set formation parameters (called by MissionCoordinator)."""
        with self._lock:
            self._formation_id = formation_id
            self._formation_type = formation_type
            self._formation_role = role
            self._leader_id = leader_id
            self._formation_offset = offset
            self._position_index = position_index
            self._action = DroneAction.FORMATION
            self._action_id += 1

    def goto(self, north: float, east: float, down: float, yaw: float = 0.0) -> None:
        """Command drone to go to position."""
        with self._lock:
            self._target = TargetPosition(north=north, east=east, down=down, yaw_deg=yaw)
            self._action = DroneAction.GOTO
            self._action_id += 1

    def destroy_node(self):
        """Clean up on shutdown."""
        self.get_logger().info(f"Shutting down DroneNode {self.drone_id}...")
        self.neighbor_tracker.stop()
        if self.mavlink:
            self.mavlink.close()
        super().destroy_node()


def main(args=None):
    """Entry point for drone_node."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    rclpy.init(args=args)

    try:
        node = DroneNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"DroneNode failed: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
