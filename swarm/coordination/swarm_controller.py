"""Main swarm orchestrator using pymavlink.

This controller wraps pymavlink connections for reliable multi-drone control
via port isolation. Integrates formations, leader-follower, missions, and
failure handling.

Based on patterns from scripts/fly_swarm_pymavlink.py.
"""

import time
import logging
from dataclasses import dataclass
from typing import Optional, List, Tuple

from pymavlink import mavutil

from .formations import (
    FormationCalculator,
    FormationType,
    FormationConfig,
    FormationTransition,
    PositionNED,
)
from .leader_follower import LeaderFollowerController
from .missions import SwarmMission, MissionExecutor
from .failure_handler import FailureHandler, FailureEvent

logger = logging.getLogger(__name__)


@dataclass
class SwarmConfig:
    """Configuration for SwarmController.

    Attributes:
        num_drones: Number of drones in swarm
        base_port: Starting UDP port for drone connections
        heartbeat_timeout: Seconds without heartbeat before failure
        position_update_rate: Position command rate in Hz
        connection_timeout: Timeout for initial connection
        ekf_timeout: Timeout for EKF convergence
        enable_vio: Enable Visual-Inertial Odometry for GPS-denied navigation
        vio_mode: VIO navigation mode ("gps", "vio", "hybrid")
    """
    num_drones: int = 3
    base_port: int = 14540
    heartbeat_timeout: float = 3.0
    position_update_rate: float = 4.0
    connection_timeout: float = 30.0
    ekf_timeout: float = 60.0
    enable_vio: bool = False
    vio_mode: str = "hybrid"  # "gps", "vio", "hybrid"


class SwarmController:
    """Main orchestrator for drone swarm operations.

    Uses pymavlink for reliable port-isolated connections to SITL.
    Coordinates formations, leader-follower, missions, and failure handling.

    Example:
        controller = SwarmController(SwarmConfig(num_drones=6))
        controller.connect_all()
        controller.wait_for_ekf_all()
        controller.set_mode_all('GUIDED')
        controller.arm_all()
        controller.takeoff_all(altitude=10.0)

        controller.fly_formation(FormationType.V_FORMATION, duration=30.0)

        controller.land_all()
        controller.disconnect_all()
    """

    def __init__(self, config: Optional[SwarmConfig] = None):
        """Initialize swarm controller.

        Args:
            config: Swarm configuration (uses defaults if None)
        """
        self.config = config or SwarmConfig()

        # MAVLink connections (one per drone, isolated by port)
        self._connections: dict[int, mavutil.mavlink_connection] = {}

        # Subsystems
        self.formation = FormationCalculator()
        self.leader_follower = LeaderFollowerController(
            self.config.num_drones,
            heartbeat_timeout=self.config.heartbeat_timeout,
        )
        self.failure_handler = FailureHandler(
            self.config.num_drones,
            heartbeat_timeout=self.config.heartbeat_timeout,
        )

        # State tracking
        self._is_connected = False
        self._is_armed = False
        self._current_positions: dict[int, PositionNED] = {}

        # VIO estimators (optional, for GPS-denied navigation)
        self._vio_estimators: dict[int, "PositionSource"] = {}
        self._vio_enabled = self.config.enable_vio

        # Register failure handler callback
        self.failure_handler.on_failure(self._handle_failure)

    # ----- Connection Management -----

    def connect_all(self, timeout: Optional[float] = None) -> bool:
        """Connect to all drones.

        Args:
            timeout: Connection timeout per drone (uses config default if None)

        Returns:
            True if all drones connected successfully
        """
        timeout = timeout or self.config.connection_timeout

        logger.info(f"Connecting to {self.config.num_drones} drones...")
        print(f"\n{'='*60}")
        print(f"Connecting to {self.config.num_drones} drones")
        print(f"{'='*60}\n")

        for i in range(self.config.num_drones):
            port = self.config.base_port + i
            try:
                conn = self._create_connection(port, timeout)
                self._connections[i] = conn
                logger.info(f"Drone {i}: Connected on port {port}")
                print(f"  Drone {i}: Connected on port {port}")
            except Exception as e:
                logger.error(f"Drone {i}: Connection failed - {e}")
                print(f"  Drone {i}: Connection FAILED - {e}")
                return False

        self._is_connected = True
        logger.info("All drones connected")
        print("\nAll drones connected.\n")
        return True

    def _create_connection(
        self,
        port: int,
        timeout: float
    ) -> mavutil.mavlink_connection:
        """Create MAVLink connection to a SITL instance.

        Args:
            port: UDP port to connect to
            timeout: Connection timeout

        Returns:
            MAVLink connection object

        Raises:
            TimeoutError: If no heartbeat received within timeout
        """
        print(f"  Connecting to udpin:0.0.0.0:{port}...")
        conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}')

        start = time.time()
        while time.time() - start < timeout:
            msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                return conn

        raise TimeoutError(f"No heartbeat on port {port} within {timeout}s")

    def disconnect_all(self) -> None:
        """Disconnect from all drones."""
        # Stop VIO estimators
        for vio in self._vio_estimators.values():
            try:
                vio.stop()
            except Exception:
                pass
        self._vio_estimators.clear()

        for conn in self._connections.values():
            try:
                conn.close()
            except Exception:
                pass

        self._connections.clear()
        self._is_connected = False
        logger.info("All drones disconnected")
        print("All drones disconnected.")

    # ----- VIO (GPS-Denied Navigation) -----

    def init_vio(self, drone_id: int) -> bool:
        """Initialize VIO for a specific drone.

        Must be called after takeoff to get initial GPS position.

        Args:
            drone_id: Drone ID to initialize VIO for

        Returns:
            True if VIO initialized successfully
        """
        if not self._vio_enabled:
            logger.warning("VIO is not enabled in config")
            return False

        if drone_id not in self._connections:
            logger.error(f"Drone {drone_id} not connected")
            return False

        try:
            from swarm.navigation import NavigationConfig, NavigationMode, PositionSource

            # Create config based on mode
            mode_map = {
                "gps": NavigationMode.GPS,
                "vio": NavigationMode.VIO,
                "hybrid": NavigationMode.HYBRID,
            }
            mode = mode_map.get(self.config.vio_mode, NavigationMode.HYBRID)

            config = NavigationConfig.for_simulation()
            config.mode = mode

            # Create position source
            source = PositionSource(config, drone_id=drone_id)

            # Get initial position from MAVLink
            conn = self._connections[drone_id]
            pos = self._get_position(conn, timeout=2.0)
            if pos is None:
                logger.error(f"Could not get initial position for drone {drone_id}")
                return False

            # Get attitude
            msg = conn.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            if msg:
                attitude = [msg.roll, msg.pitch, msg.yaw]
            else:
                attitude = [0.0, 0.0, 0.0]

            # Initialize
            altitude = -pos[2]  # Down to up
            timestamp_us = int(time.time() * 1e6)

            success = source.initialize(
                position=[pos[0], pos[1], pos[2]],
                velocity=[0.0, 0.0, 0.0],
                attitude=attitude,
                altitude=altitude,
                timestamp_us=timestamp_us,
            )

            if success:
                source.set_mavlink_connection(conn)
                source.start()
                self._vio_estimators[drone_id] = source
                logger.info(f"VIO initialized for drone {drone_id}")
                return True
            else:
                logger.error(f"Failed to initialize VIO for drone {drone_id}")
                return False

        except ImportError as e:
            logger.error(f"Failed to import navigation module: {e}")
            return False
        except Exception as e:
            logger.error(f"Error initializing VIO for drone {drone_id}: {e}")
            return False

    def init_vio_all(self) -> bool:
        """Initialize VIO for all connected drones.

        Returns:
            True if all drones initialized successfully
        """
        if not self._vio_enabled:
            logger.warning("VIO is not enabled in config")
            return False

        all_success = True
        for drone_id in self._connections:
            if not self.init_vio(drone_id):
                all_success = False

        return all_success

    def deny_gps(self, drone_id: int) -> None:
        """Simulate GPS denial for a drone (testing only).

        Args:
            drone_id: Drone to deny GPS
        """
        if drone_id in self._vio_estimators:
            self._vio_estimators[drone_id].deny_gps()
            logger.info(f"GPS denied for drone {drone_id}")

    def restore_gps(self, drone_id: int) -> None:
        """Restore GPS for a drone (testing only).

        Args:
            drone_id: Drone to restore GPS
        """
        if drone_id in self._vio_estimators:
            self._vio_estimators[drone_id].restore_gps()
            logger.info(f"GPS restored for drone {drone_id}")

    def get_vio_state(self, drone_id: int):
        """Get VIO state for a drone.

        Args:
            drone_id: Drone ID

        Returns:
            VIOState or None
        """
        if drone_id in self._vio_estimators:
            return self._vio_estimators[drone_id].get_state()
        return None

    # ----- Basic Flight Commands -----

    def wait_for_ekf_all(self, timeout: Optional[float] = None) -> bool:
        """Wait for EKF convergence on all drones.

        EKF convergence is required before arming to get valid position estimates.

        Args:
            timeout: Maximum wait time per drone

        Returns:
            True if all drones have valid position estimates
        """
        timeout = timeout or self.config.ekf_timeout

        logger.info("Waiting for EKF convergence...")
        print("Waiting for EKF convergence...")

        for drone_id, conn in self._connections.items():
            if not self._wait_for_ekf(conn, drone_id, timeout):
                logger.error(f"Drone {drone_id}: EKF timeout")
                print(f"  Drone {drone_id}: EKF TIMEOUT")
                return False

        logger.info("All drones have valid position estimates")
        print("All drones have valid position estimates.\n")
        return True

    def _wait_for_ekf(
        self,
        conn: mavutil.mavlink_connection,
        drone_id: int,
        timeout: float
    ) -> bool:
        """Wait for EKF convergence on single drone.

        EKF_STATUS_REPORT flags:
        - bit 3 (0x08): const_pos_mode
        - bit 4 (0x10): pred_pos_horiz_abs
        """
        print(f"  Waiting for EKF on drone {drone_id}...")
        start = time.time()

        while time.time() - start < timeout:
            msg = conn.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
            if msg and (msg.flags & 0x18) != 0:
                print(f"  Drone {drone_id}: EKF converged (flags=0x{msg.flags:02x})")
                return True

        return False

    def set_mode_all(self, mode: str) -> bool:
        """Set flight mode on all drones.

        Args:
            mode: Flight mode name (e.g., 'GUIDED', 'LAND', 'RTL')

        Returns:
            True if all drones changed mode successfully
        """
        logger.info(f"Setting {mode} mode on all drones...")
        print(f"Setting {mode} mode...")

        for drone_id, conn in self._connections.items():
            if not self._set_mode(conn, drone_id, mode):
                logger.error(f"Drone {drone_id}: Failed to set {mode} mode")
                print(f"  Drone {drone_id}: Mode change FAILED")
                return False
            print(f"  Drone {drone_id}: {mode} mode set")

        return True

    def _set_mode(
        self,
        conn: mavutil.mavlink_connection,
        drone_id: int,
        mode: str
    ) -> bool:
        """Set flight mode on single drone."""
        mode_mapping = conn.mode_mapping()
        if mode not in mode_mapping:
            logger.error(f"Unknown mode: {mode}")
            return False

        mode_id = mode_mapping[mode]
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        start = time.time()
        while time.time() - start < 5.0:
            msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                return True

        return False

    def arm_all(self) -> bool:
        """Arm all drones.

        Returns:
            True if all drones armed successfully
        """
        logger.info("Arming all drones...")
        print("Arming drones...")

        for drone_id, conn in self._connections.items():
            if not self._arm(conn, drone_id):
                logger.error(f"Drone {drone_id}: Arm failed")
                print(f"  Drone {drone_id}: Arm FAILED")
                return False
            print(f"  Drone {drone_id}: Armed")

        self._is_armed = True
        logger.info("All drones armed")
        print("All drones armed.\n")
        return True

    def _arm(
        self,
        conn: mavutil.mavlink_connection,
        drone_id: int,
        timeout: float = 10.0
    ) -> bool:
        """Arm single drone."""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

        start = time.time()
        while time.time() - start < timeout:
            msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True

        return False

    def disarm_all(self) -> None:
        """Disarm all drones."""
        logger.info("Disarming all drones...")
        print("Disarming drones...")

        for drone_id, conn in self._connections.items():
            self._disarm(conn, drone_id)
            print(f"  Drone {drone_id}: Disarmed")

        self._is_armed = False

    def _disarm(self, conn: mavutil.mavlink_connection, drone_id: int) -> None:
        """Disarm single drone."""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    def takeoff_all(self, altitude: float = 10.0, wait: bool = True) -> bool:
        """Command takeoff for all drones.

        Args:
            altitude: Target altitude in meters
            wait: If True, wait for drones to reach altitude

        Returns:
            True if takeoff commanded successfully
        """
        logger.info(f"Taking off to {altitude}m...")
        print(f"Taking off to {altitude}m...")

        for drone_id, conn in self._connections.items():
            self._takeoff(conn, drone_id, altitude)
            print(f"  Drone {drone_id}: Takeoff commanded")

        if wait:
            print("Waiting for drones to reach altitude...")
            time.sleep(10)  # Simple wait for now
            print("All drones airborne.\n")

        return True

    def _takeoff(
        self,
        conn: mavutil.mavlink_connection,
        drone_id: int,
        altitude: float
    ) -> None:
        """Command takeoff on single drone."""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # pitch
            0,  # empty
            0,  # empty
            0,  # yaw
            0,  # latitude
            0,  # longitude
            altitude
        )

    def land_all(self) -> None:
        """Command landing for all drones."""
        logger.info("Landing all drones...")
        print("\nLanding all drones...")

        for drone_id, conn in self._connections.items():
            self._land(conn, drone_id)
            print(f"  Drone {drone_id}: Land commanded")

        print("Waiting for landing...")
        time.sleep(15)
        print("Landing complete.\n")

    def _land(self, conn: mavutil.mavlink_connection, drone_id: int) -> None:
        """Command land on single drone."""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # confirmation
            0,  # abort altitude
            0,  # land mode
            0,  # empty
            0,  # yaw
            0,  # latitude
            0,  # longitude
            0   # altitude
        )

    def return_to_launch_all(self) -> None:
        """Command return to launch for all drones."""
        logger.info("RTL all drones...")
        print("Commanding RTL...")

        for drone_id, conn in self._connections.items():
            self._set_mode(conn, drone_id, 'RTL')
            print(f"  Drone {drone_id}: RTL commanded")

    # ----- Position Control -----

    def _send_position_ned(
        self,
        conn: mavutil.mavlink_connection,
        position: PositionNED,
        yaw: float = 0.0
    ) -> None:
        """Send position setpoint in local NED frame.

        Args:
            conn: MAVLink connection
            position: Target (north, east, down) position
            yaw: Target yaw angle in degrees
        """
        # Type mask: position only, ignore velocity/acceleration
        type_mask = 0b0000111111111000

        conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            position[0], position[1], position[2],  # north, east, down
            0, 0, 0,  # velocity (ignored)
            0, 0, 0,  # acceleration (ignored)
            yaw, 0    # yaw, yaw_rate
        )

    def _get_position(
        self,
        conn: mavutil.mavlink_connection,
        timeout: float = 1.0,
        drone_id: Optional[int] = None
    ) -> Optional[PositionNED]:
        """Get current position from drone.

        If VIO is enabled and GPS is denied, uses VIO position estimate.
        Otherwise falls back to ArduPilot's LOCAL_POSITION_NED.

        Args:
            conn: MAVLink connection
            timeout: How long to wait for position message
            drone_id: Drone ID for VIO lookup (optional)

        Returns:
            (north, east, down) position or None if unavailable
        """
        # Try VIO first if enabled
        if self._vio_enabled and drone_id is not None:
            if drone_id in self._vio_estimators:
                pos, mode = self._vio_estimators[drone_id].get_position()
                if pos is not None:
                    # VIO provides valid position
                    from swarm.navigation import NavigationMode
                    if mode != NavigationMode.DEAD_RECKONING:
                        return (pos[0], pos[1], pos[2])
                    # Dead reckoning - fall back to MAVLink

        # Standard MAVLink position
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
        if msg:
            return (msg.x, msg.y, msg.z)
        return None

    def _get_all_positions(self, timeout: float = 0.5) -> dict[int, PositionNED]:
        """Get current positions from all connected drones.

        Args:
            timeout: How long to wait per drone

        Returns:
            Dict mapping drone_id to (north, east, down) position
        """
        positions = {}
        for drone_id, conn in self._connections.items():
            pos = self._get_position(conn, timeout)
            if pos:
                positions[drone_id] = pos
        return positions

    def _distance_to_target(
        self,
        current: PositionNED,
        target: PositionNED
    ) -> float:
        """Calculate 3D distance between current and target positions.

        Args:
            current: Current (n, e, d) position
            target: Target (n, e, d) position

        Returns:
            Distance in meters
        """
        return (
            (current[0] - target[0]) ** 2 +
            (current[1] - target[1]) ** 2 +
            (current[2] - target[2]) ** 2
        ) ** 0.5

    def _wait_for_positions(
        self,
        targets: dict[int, PositionNED],
        tolerance: float = 2.0,
        timeout: float = 30.0,
    ) -> bool:
        """Wait for drones to reach target positions.

        Continuously sends position commands while waiting for arrival.

        Args:
            targets: Dict mapping drone_id to target (n, e, d) position
            tolerance: Distance threshold for "arrived" (meters)
            timeout: Maximum time to wait (seconds)

        Returns:
            True if all drones reached their targets within timeout
        """
        start = time.time()
        interval = 1.0 / self.config.position_update_rate
        arrived = {drone_id: False for drone_id in targets}

        while time.time() - start < timeout:
            # Send position commands to keep drones moving
            for drone_id, target in targets.items():
                if drone_id in self._connections:
                    self._send_position_ned(self._connections[drone_id], target)
                    # Keep heartbeat alive
                    self.failure_handler.update_heartbeat(drone_id, target)

            # Check positions for each drone
            all_arrived = True
            for drone_id, target in targets.items():
                if arrived[drone_id]:
                    continue  # Already arrived

                if drone_id not in self._connections:
                    continue

                pos = self._get_position(self._connections[drone_id], timeout=0.3)
                if pos:
                    dist = self._distance_to_target(pos, target)
                    if dist <= tolerance:
                        arrived[drone_id] = True
                        logger.debug(f"Drone {drone_id} arrived at target (dist={dist:.1f}m)")
                    else:
                        all_arrived = False
                else:
                    all_arrived = False

            if all_arrived:
                logger.debug("All drones reached target positions")
                return True

            time.sleep(interval)

        # Log which drones didn't arrive
        for drone_id, did_arrive in arrived.items():
            if not did_arrive:
                logger.warning(f"Drone {drone_id} did not reach target within {timeout}s")

        return False

    # ----- Formation Flying -----

    def fly_formation(
        self,
        formation_type: FormationType,
        duration: float = 15.0,
        config: Optional[FormationConfig] = None,
        arrival_timeout: float = 30.0,
        arrival_tolerance: float = 2.0,
    ) -> bool:
        """Fly a formation for specified duration.

        First waits for all drones to reach their formation positions,
        then holds the formation for the specified duration.

        Args:
            formation_type: Type of formation to fly
            duration: How long to hold formation after arrival (seconds)
            config: Formation configuration
            arrival_timeout: Max time to wait for drones to reach positions
            arrival_tolerance: Distance threshold for "arrived" (meters)

        Returns:
            True if formation flew successfully
        """
        # Refresh heartbeats before checking health to avoid false positives
        # from gaps between formation commands
        for drone_id in range(self.config.num_drones):
            if drone_id not in self.failure_handler._failed_drones:
                self.failure_handler.update_heartbeat(drone_id)

        active_drones = self.failure_handler.get_active_drones()
        num_active = len(active_drones)

        if num_active == 0:
            logger.error("No active drones for formation")
            return False

        # Calculate positions for active drones
        positions = self.formation.calculate(formation_type, num_active, config)

        logger.info(f"Flying {formation_type.value} formation with {num_active} drones")
        print(f"\nFlying {formation_type.value.upper()} formation...")

        for idx, drone_id in enumerate(active_drones):
            n, e, d = positions[idx]
            print(f"  Drone {drone_id}: N={n:.1f}, E={e:.1f}, Alt={-d:.1f}m")

        # Build target positions dict
        targets = {
            active_drones[idx]: positions[idx]
            for idx in range(len(active_drones))
        }

        # Phase 1: Wait for drones to reach formation positions
        print(f"  Waiting for drones to reach positions (tolerance={arrival_tolerance}m)...")
        if not self._wait_for_positions(targets, arrival_tolerance, arrival_timeout):
            logger.warning("Some drones did not reach formation positions in time")
            print("  WARNING: Some drones did not reach positions in time")
            # Continue anyway - partial formation is better than nothing

        print(f"  Holding formation for {duration}s...")

        # Phase 2: Hold formation for duration
        interval = 1.0 / self.config.position_update_rate
        start = time.time()

        while time.time() - start < duration:
            # Check health
            failures = self.failure_handler.check_all_health()

            # Handle any new failures
            new_active = self.failure_handler.get_active_drones()
            if len(new_active) < num_active:
                logger.warning(f"Drones reduced: {num_active} -> {len(new_active)}")
                print(f"  WARNING: {num_active - len(new_active)} drone(s) failed")

                # Recalculate formation for remaining drones
                positions = self.formation.calculate(
                    formation_type, len(new_active), config
                )
                active_drones = new_active
                num_active = len(new_active)

                # Update targets
                targets = {
                    active_drones[idx]: positions[idx]
                    for idx in range(len(active_drones))
                }

            # Send position commands to all active drones
            for idx, drone_id in enumerate(active_drones):
                if drone_id in self._connections:
                    self._send_position_ned(self._connections[drone_id], positions[idx])

                    # Update failure handler with heartbeat
                    self.failure_handler.update_heartbeat(drone_id, positions[idx])

            time.sleep(interval)

        logger.info(f"Formation {formation_type.value} complete")
        print(f"Formation {formation_type.value} complete.\n")
        return True

    def fly_formation_transition(
        self,
        from_type: FormationType,
        to_type: FormationType,
        transition_duration: float = 5.0,
        hold_duration: float = 10.0,
        config: Optional[FormationConfig] = None,
    ) -> bool:
        """Smoothly transition between two formations.

        Args:
            from_type: Starting formation
            to_type: Target formation
            transition_duration: Time to transition (seconds)
            hold_duration: Time to hold final formation (seconds)
            config: Formation configuration

        Returns:
            True if transition completed successfully
        """
        # Refresh heartbeats before checking health
        for drone_id in range(self.config.num_drones):
            if drone_id not in self.failure_handler._failed_drones:
                self.failure_handler.update_heartbeat(drone_id)

        active_drones = self.failure_handler.get_active_drones()
        num_active = len(active_drones)

        if num_active == 0:
            return False

        start_positions = self.formation.calculate(from_type, num_active, config)
        end_positions = self.formation.calculate(to_type, num_active, config)

        transition = FormationTransition(
            start_positions=start_positions,
            end_positions=end_positions,
            duration=transition_duration,
        )

        logger.info(f"Transitioning from {from_type.value} to {to_type.value}")
        print(f"\nTransitioning: {from_type.value} -> {to_type.value}...")

        interval = 1.0 / self.config.position_update_rate
        start = time.time()

        # Transition phase
        while not transition.is_complete(time.time() - start):
            positions = transition.get_positions_at_time(time.time() - start)

            for idx, drone_id in enumerate(active_drones):
                if drone_id in self._connections:
                    self._send_position_ned(self._connections[drone_id], positions[idx])
                    # Keep heartbeat alive during transition
                    self.failure_handler.update_heartbeat(drone_id, positions[idx])

            time.sleep(interval)

        print(f"Transition complete. Holding {to_type.value}...")

        # Hold final formation
        return self.fly_formation(to_type, duration=hold_duration, config=config)

    # ----- Leader-Follower Mode -----

    def start_leader_follower(
        self,
        leader_id: int = 0,
        formation_type: FormationType = FormationType.V_FORMATION,
        config: Optional[FormationConfig] = None,
    ) -> None:
        """Start leader-follower mode.

        Args:
            leader_id: ID of leader drone
            formation_type: Formation for followers
            config: Formation configuration
        """
        # Refresh heartbeats before checking active drones
        for drone_id in range(self.config.num_drones):
            if drone_id not in self.failure_handler._failed_drones:
                self.failure_handler.update_heartbeat(drone_id)

        active_drones = self.failure_handler.get_active_drones()

        # Calculate formation and convert to offsets
        positions = self.formation.calculate(
            formation_type, len(active_drones), config
        )
        self.leader_follower.set_offsets_from_positions(positions, leader_id)
        self.leader_follower.set_leader(leader_id)

        logger.info(f"Leader-follower started with drone {leader_id} as leader")
        print(f"\nLeader-follower mode started. Leader: drone {leader_id}")

    def update_leader_follower(
        self,
        leader_position: PositionNED,
        duration: Optional[float] = None,
        wait_for_arrival: bool = True,
        arrival_timeout: float = 20.0,
        arrival_tolerance: float = 2.0,
    ) -> None:
        """Update follower positions based on leader position.

        Call this in a loop while leader-follower is active.

        Args:
            leader_position: Leader's current (north, east, down) position
            duration: Optional duration to hold after arrival (seconds). If None, runs once.
            wait_for_arrival: If True, wait for all drones to reach positions before starting hold
            arrival_timeout: Max time to wait for drones to reach positions
            arrival_tolerance: Distance threshold for "arrived" (meters)
        """
        interval = 1.0 / self.config.position_update_rate

        if duration is None:
            # Single update
            self._update_followers_once(leader_position)
            return

        # Refresh heartbeats before starting duration loop
        for drone_id in range(self.config.num_drones):
            if drone_id not in self.failure_handler._failed_drones:
                self.failure_handler.update_heartbeat(drone_id)

        # Build target positions for all drones (leader + followers)
        leader_id = self.leader_follower.leader_id
        targets = self.leader_follower.get_all_target_positions(leader_position)
        if leader_id is not None:
            targets[leader_id] = leader_position

        # Wait for drones to reach positions first
        if wait_for_arrival and targets:
            self._wait_for_positions(targets, arrival_tolerance, arrival_timeout)

        # Hold positions for duration
        start = time.time()
        while time.time() - start < duration:
            # Check leader health
            if not self.leader_follower.check_leader_health():
                logger.error("No healthy leader available")
                break

            self._update_followers_once(leader_position)
            time.sleep(interval)

    def _update_followers_once(self, leader_position: PositionNED) -> None:
        """Send position commands to leader and all followers."""
        # Command the leader to move to the specified position
        leader_id = self.leader_follower.leader_id
        if leader_id is not None and leader_id in self._connections:
            self._send_position_ned(self._connections[leader_id], leader_position)
            self.failure_handler.update_heartbeat(leader_id, leader_position)

        # Command followers to their offset positions
        targets = self.leader_follower.get_all_target_positions(leader_position)

        for drone_id, pos in targets.items():
            if drone_id in self._connections:
                self._send_position_ned(self._connections[drone_id], pos)
                # Keep heartbeat alive
                self.failure_handler.update_heartbeat(drone_id, pos)

    # ----- Mission Execution -----

    def execute_mission(self, mission: SwarmMission) -> bool:
        """Execute a swarm mission.

        Args:
            mission: SwarmMission to execute

        Returns:
            True if mission completed successfully
        """
        # Refresh heartbeats before starting
        for drone_id in range(self.config.num_drones):
            if drone_id not in self.failure_handler._failed_drones:
                self.failure_handler.update_heartbeat(drone_id)

        executor = MissionExecutor(mission)
        executor.start()

        logger.info(f"Executing mission: {mission.name}")
        print(f"\nExecuting mission: {mission.name}")
        print(f"Description: {mission.description}")

        interval = 1.0 / self.config.position_update_rate

        while not mission.is_complete():
            # Check health
            failures = self.failure_handler.check_all_health()
            if failures:
                for f in failures:
                    logger.warning(f"Mission affected by failure: drone {f.drone_id}")
                    print(f"  WARNING: Drone {f.drone_id} failed during mission")

            # Get current targets
            targets = executor.get_current_targets()

            # Send position commands
            for drone_id, pos in targets.items():
                if drone_id in self._connections:
                    conn = self._connections[drone_id]
                    self._send_position_ned(conn, pos)

                    # Get actual position for waypoint arrival detection
                    actual_pos = self._get_position(conn, timeout=0.3)
                    if actual_pos:
                        executor.update_drone_position(drone_id, actual_pos)
                        self.failure_handler.update_heartbeat(drone_id, actual_pos)
                    else:
                        # Fall back to target if no position available
                        self.failure_handler.update_heartbeat(drone_id, pos)

            # Progress update
            progress = mission.get_progress()
            if int(progress * 100) % 25 == 0:
                logger.debug(f"Mission progress: {progress*100:.0f}%")

            time.sleep(interval)

        executor.stop()
        logger.info(f"Mission {mission.name} complete")
        print(f"Mission {mission.name} complete.\n")
        return True

    # ----- Failure Handling -----

    def _handle_failure(self, event: FailureEvent) -> None:
        """Handle drone failure event.

        Args:
            event: Failure event details
        """
        logger.warning(f"Handling failure: drone {event.drone_id}")
        print(f"  FAILURE: Drone {event.drone_id} - {event.failure_type.value}")

        # If leader failed in leader-follower mode, controller will auto-promote
        if event.drone_id == self.leader_follower.leader_id:
            self.leader_follower.check_leader_health()

    # ----- Status and Properties -----

    @property
    def is_connected(self) -> bool:
        """Whether all drones are connected."""
        return self._is_connected

    @property
    def is_armed(self) -> bool:
        """Whether all drones are armed."""
        return self._is_armed

    def get_active_drone_count(self) -> int:
        """Get count of active (healthy) drones."""
        return len(self.failure_handler.get_active_drones())

    def get_connection(self, drone_id: int) -> Optional[mavutil.mavlink_connection]:
        """Get MAVLink connection for a specific drone.

        Args:
            drone_id: Drone identifier

        Returns:
            MAVLink connection or None if not connected
        """
        return self._connections.get(drone_id)
