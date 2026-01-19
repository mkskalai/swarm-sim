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
        spawn_layout: Spawn layout used in world gen ("grid" or "line")
        spawn_spacing: Spacing between drones at spawn (meters)
        collision_avoidance_separation: Vertical separation for collision avoidance (meters)
    """
    num_drones: int = 3
    base_port: int = 14540
    heartbeat_timeout: float = 3.0
    position_update_rate: float = 4.0
    connection_timeout: float = 30.0
    ekf_timeout: float = 60.0
    enable_vio: bool = False
    vio_mode: str = "hybrid"  # "gps", "vio", "hybrid"
    spawn_layout: str = "grid"
    spawn_spacing: float = 5.0
    collision_avoidance_separation: float = 3.0  # meters between drones during transitions


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

        controller.fly_formation(FormationType.LINE, duration=30.0)

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

        # Home position offsets (NED) for each drone based on spawn positions
        # Used to transform world formations to drone-local coordinates
        self._home_offsets: dict[int, PositionNED] = self._calculate_home_offsets()

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

    def land_all(
        self,
        timeout: Optional[float] = None,
        altitude_threshold: float = 0.5,
        verify: bool = True,
    ) -> bool:
        """Command landing for all drones.

        Args:
            timeout: Max time to wait for landing (default 60s, scaled by timeout_multiplier)
            altitude_threshold: Altitude threshold for "landed" (meters)
            verify: If True, verify all drones reached ground

        Returns:
            True if all drones landed (or verify=False)
        """
        logger.info("Landing all drones...")
        print("\nLanding all drones...")

        for drone_id, conn in self._connections.items():
            self._land(conn, drone_id)
            print(f"  Drone {drone_id}: Land commanded")

        if not verify:
            print("Waiting for landing (no verification)...")
            time.sleep(15)
            print("Landing complete.\n")
            return True

        # Verify landing with timeout
        if timeout is None:
            timeout = 60.0

        print(f"Verifying landing (timeout={timeout}s, threshold={altitude_threshold}m)...")
        landed = self._verify_landed(timeout, altitude_threshold)

        if landed:
            print("Landing complete - all drones verified on ground.\n")
        else:
            print("WARNING: Not all drones verified landed.\n")

        return landed

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

    def _verify_landed(
        self,
        timeout: float = 60.0,
        altitude_threshold: float = 0.5,
    ) -> bool:
        """Verify all drones have landed.

        Polls position until all drones report altitude below threshold.

        Args:
            timeout: Max time to wait (seconds)
            altitude_threshold: Max altitude to consider "landed" (meters)

        Returns:
            True if all drones below threshold within timeout
        """
        start = time.time()
        landed = {drone_id: False for drone_id in self._connections}

        while time.time() - start < timeout:
            all_landed = True

            for drone_id, conn in self._connections.items():
                if landed[drone_id]:
                    continue  # Already confirmed landed

                pos = self._get_position(conn, timeout=0.5, drone_id=drone_id)
                if pos:
                    altitude = -pos[2]  # Convert Down to altitude
                    if altitude < altitude_threshold:
                        landed[drone_id] = True
                        logger.debug(f"Drone {drone_id} landed (alt={altitude:.2f}m)")
                    else:
                        all_landed = False
                else:
                    all_landed = False

            if all_landed:
                return True

            time.sleep(0.5)

        # Log which drones didn't land
        for drone_id, is_landed in landed.items():
            if not is_landed:
                logger.warning(f"Drone {drone_id} did not verify landed within {timeout}s")

        return False

    def spread_and_land(
        self,
        spacing: float = 8.0,
        spread_timeout: float = 30.0,
        landing_timeout: float = 60.0,
        altitude_threshold: float = 0.5,
    ) -> bool:
        """Spread drones to spawn formation then land to avoid collision.

        Moves all drones back to their spawn positions (same layout as initial
        spawn: GRID or LINE) at current average altitude, then commands landing
        with verification. This prevents drones from colliding during descent.

        Args:
            spacing: Horizontal spacing between drones (meters)
            spread_timeout: Time to reach spread positions (seconds)
            landing_timeout: Time to land and verify (seconds)
            altitude_threshold: Altitude threshold for "landed" (meters)

        Returns:
            True if all drones landed successfully
        """
        layout = self.config.spawn_layout
        logger.info(f"Spreading drones to {layout} formation before landing...")
        print(f"\nSpreading drones to {layout.upper()} formation before landing...")

        # Get current positions and calculate average altitude
        positions = self._get_all_positions(timeout=1.0)
        if not positions:
            logger.warning("Could not get positions, falling back to direct landing")
            return self.land_all(timeout=landing_timeout, verify=True)

        avg_altitude = sum(-pos[2] for pos in positions.values()) / len(positions)
        logger.debug(f"Average altitude: {avg_altitude:.1f}m")

        # Calculate spread positions based on spawn layout (world coords)
        # Sort drones by current position to prevent path crossing during spread
        num_drones = len(positions)
        spread_targets = {}

        # Convert local positions to world positions for sorting
        world_positions = {}
        for drone_id, local_pos in positions.items():
            home = self._home_offsets.get(drone_id, (0, 0, 0))
            world_pos = (
                local_pos[0] + home[0],
                local_pos[1] + home[1],
                local_pos[2] + home[2],
            )
            world_positions[drone_id] = world_pos

        if layout == "line":
            # LINE: spread along East axis
            # Sort by current East position to prevent crossing
            sorted_drones = sorted(positions.keys(), key=lambda d: world_positions[d][1])
            center_offset = (num_drones - 1) / 2
            for idx, drone_id in enumerate(sorted_drones):
                east = (idx - center_offset) * spacing
                world_pos = (0.0, east, -avg_altitude)
                local_pos = self._world_to_local(world_pos, drone_id)
                spread_targets[drone_id] = local_pos
                print(f"  Drone {drone_id}: spread to world N=0, E={east:.1f}m")
        else:
            # GRID: spread to grid pattern (matching spawn)
            # Sort by row-major (North descending, East ascending) to prevent crossing
            import math
            sorted_drones = sorted(
                positions.keys(),
                key=lambda d: (-world_positions[d][0], world_positions[d][1])
            )
            cols = math.ceil(math.sqrt(num_drones))
            grid_east_offset = (cols - 1) * spacing / 2
            rows = math.ceil(num_drones / cols)
            grid_north_offset = (rows - 1) * spacing / 2

            for idx, drone_id in enumerate(sorted_drones):
                row, col = divmod(idx, cols)
                north = grid_north_offset - row * spacing
                east = col * spacing - grid_east_offset
                world_pos = (north, east, -avg_altitude)
                local_pos = self._world_to_local(world_pos, drone_id)
                spread_targets[drone_id] = local_pos
                print(f"  Drone {drone_id}: spread to world N={north:.1f}, E={east:.1f}m")

        # Move to spread positions with collision avoidance
        # Uses 3-phase altitude separation to prevent path crossing during spread
        spread_ok = self._fly_with_collision_avoidance(
            spread_targets,
            tolerance=2.0,
            timeout=spread_timeout,
        )

        if not spread_ok:
            logger.warning("Not all drones reached spread positions, landing anyway")
            print("  WARNING: Some drones did not reach spread positions")

        # Now land
        return self.land_all(
            timeout=landing_timeout,
            altitude_threshold=altitude_threshold,
            verify=True,
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

    def goto_all(
        self,
        north: float,
        east: float,
        altitude: float,
        wait: bool = True,
        timeout: float = 30.0,
        tolerance: float = 2.0,
    ) -> bool:
        """Command all drones to fly to a position.

        All drones fly to the same world position (useful for rallying).
        Coordinates are transformed to each drone's local frame.

        Args:
            north: North position in meters (world frame)
            east: East position in meters (world frame)
            altitude: Altitude in meters (positive = up)
            wait: If True, wait for drones to arrive
            timeout: Maximum wait time for arrival
            tolerance: Distance threshold for "arrived" (meters)

        Returns:
            True if all drones reached position (or wait=False)
        """
        # Convert altitude to down coordinate (NED)
        down = -altitude
        world_target = (north, east, down)

        logger.info(f"Moving all drones to N={north:.1f}, E={east:.1f}, Alt={altitude:.1f}m")
        print(f"Moving all drones to N={north:.1f}, E={east:.1f}, Alt={altitude:.1f}m...")

        # Transform world position to each drone's local frame
        targets = {
            drone_id: self._world_to_local(world_target, drone_id)
            for drone_id in self._connections
        }

        if wait:
            return self._wait_for_positions(targets, tolerance, timeout)
        else:
            # Just send the commands once
            for drone_id, conn in self._connections.items():
                self._send_position_ned(conn, targets[drone_id])
            return True

    def goto_formation(
        self,
        center_north: float,
        center_east: float,
        altitude: float,
        formation_type: Optional[FormationType] = None,
        config: Optional[FormationConfig] = None,
        wait: bool = True,
        timeout: float = 30.0,
        tolerance: float = 2.0,
    ) -> bool:
        """Move swarm to a new center position while maintaining formation.

        Unlike goto_all() which moves all drones to the same point, this method
        moves the formation CENTER to the specified position while maintaining
        the relative formation offsets between drones.

        Args:
            center_north: North position for formation center (world frame)
            center_east: East position for formation center (world frame)
            altitude: Altitude in meters (positive = up)
            formation_type: Formation to maintain (defaults to LINE if not specified)
            config: Formation configuration (spacing, etc.)
            wait: If True, wait for drones to arrive
            timeout: Maximum wait time for arrival
            tolerance: Distance threshold for "arrived" (meters)

        Returns:
            True if all drones reached positions (or wait=False)
        """
        active_drones = self.failure_handler.get_active_drones()
        num_active = len(active_drones)

        if num_active == 0:
            logger.error("No active drones for goto_formation")
            return False

        # Default to LINE formation if not specified
        formation = formation_type or FormationType.LINE
        cfg = config or FormationConfig(altitude=altitude)

        # Calculate formation positions centered at origin
        formation_positions = self.formation.calculate(formation, num_active, cfg)

        # Offset all positions to the new center
        down = -altitude
        world_positions = [
            (center_north + pos[0], center_east + pos[1], down)
            for pos in formation_positions
        ]

        logger.info(
            f"Moving {formation.value} formation to N={center_north:.1f}, "
            f"E={center_east:.1f}, Alt={altitude:.1f}m"
        )
        print(
            f"Moving {formation.value.upper()} formation to "
            f"N={center_north:.1f}, E={center_east:.1f}, Alt={altitude:.1f}m..."
        )

        # Sort drones for optimal slot assignment
        current_positions = self._get_all_positions(timeout=1.0)
        sorted_drones = self._sort_drones_for_formation(
            active_drones, current_positions, formation
        )

        # Transform world positions to each drone's local frame
        targets = {}
        for idx, drone_id in enumerate(sorted_drones):
            world_pos = world_positions[idx]
            local_pos = self._world_to_local(world_pos, drone_id)
            targets[drone_id] = local_pos

        if wait:
            return self._fly_with_collision_avoidance(targets, tolerance, timeout)
        else:
            # Just send the commands once
            for drone_id, conn in self._connections.items():
                if drone_id in targets:
                    self._send_position_ned(conn, targets[drone_id])
            return True

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

        # Calculate formation positions in world frame (relative to drone 0's home)
        world_positions = self.formation.calculate(formation_type, num_active, config)

        logger.info(f"Flying {formation_type.value} formation with {num_active} drones")
        print(f"\nFlying {formation_type.value.upper()} formation...")

        # Sort drones by current world position to assign formation slots correctly
        # This prevents path crossing when drones have different spawn positions
        current_positions = self._get_all_positions(timeout=1.0)
        sorted_drones = self._sort_drones_for_formation(
            active_drones, current_positions, formation_type
        )

        # Transform world positions to each drone's local frame
        # Formation slot i goes to sorted_drones[i], not active_drones[i]
        targets = {}
        for idx, drone_id in enumerate(sorted_drones):
            world_pos = world_positions[idx]
            local_pos = self._world_to_local(world_pos, drone_id)
            targets[drone_id] = local_pos
            print(f"  Drone {drone_id}: World N={world_pos[0]:.1f}, E={world_pos[1]:.1f} "
                  f"-> Local N={local_pos[0]:.1f}, E={local_pos[1]:.1f}, Alt={-local_pos[2]:.1f}m")

        # Phase 1: Move drones to formation positions with collision avoidance
        # Uses 3-phase altitude separation to prevent path crossing
        if not self._fly_with_collision_avoidance(targets, arrival_tolerance, arrival_timeout):
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
                world_positions = self.formation.calculate(
                    formation_type, len(new_active), config
                )
                active_drones = new_active
                num_active = len(new_active)

                # Update targets with local coordinate transformation
                targets = {}
                for idx, drone_id in enumerate(active_drones):
                    world_pos = world_positions[idx]
                    targets[drone_id] = self._world_to_local(world_pos, drone_id)

            # Send position commands to all active drones
            for drone_id in active_drones:
                if drone_id in self._connections and drone_id in targets:
                    self._send_position_ned(self._connections[drone_id], targets[drone_id])

                    # Update failure handler with heartbeat
                    self.failure_handler.update_heartbeat(drone_id, targets[drone_id])

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

        # Sort drones by current position for optimal slot assignment
        current_positions = self._get_all_positions(timeout=1.0)
        sorted_drones = self._sort_drones_for_formation(
            active_drones, current_positions, to_type
        )

        # Calculate world-frame positions for start and end formations
        start_world = self.formation.calculate(from_type, num_active, config)
        end_world = self.formation.calculate(to_type, num_active, config)

        # Transform to local coordinates for each drone (using sorted order)
        start_local = [
            self._world_to_local(start_world[idx], sorted_drones[idx])
            for idx in range(num_active)
        ]
        end_local = [
            self._world_to_local(end_world[idx], sorted_drones[idx])
            for idx in range(num_active)
        ]

        transition = FormationTransition(
            start_positions=start_local,
            end_positions=end_local,
            duration=transition_duration,
        )

        logger.info(f"Transitioning from {from_type.value} to {to_type.value}")
        print(f"\nTransitioning: {from_type.value} -> {to_type.value}...")

        interval = 1.0 / self.config.position_update_rate
        start = time.time()

        # Transition phase (use sorted_drones to match slot assignment)
        while not transition.is_complete(time.time() - start):
            positions = transition.get_positions_at_time(time.time() - start)

            for idx, drone_id in enumerate(sorted_drones):
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
        formation_type: FormationType = FormationType.LINE,
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

    # ----- Coordinate Frame Helpers -----

    def _sort_drones_for_formation(
        self,
        drone_ids: list[int],
        current_positions: dict[int, PositionNED],
        formation_type: "FormationType",
    ) -> list[int]:
        """Sort drones by position for optimal formation slot assignment.

        This prevents path crossing when transitioning to a formation.
        Drones are sorted so that the one closest to formation slot 0 gets
        slot 0, etc.

        For LINE formation: sort by East position (leftmost gets slot 0)
        For GRID formation: sort by row-major (North, then East)

        Args:
            drone_ids: List of active drone IDs
            current_positions: Dict of drone_id -> local position (NED)
            formation_type: The formation being flown

        Returns:
            List of drone IDs sorted for optimal slot assignment
        """
        from swarm.coordination import FormationType

        # Convert local positions to world positions for sorting
        world_positions = {}
        for drone_id in drone_ids:
            if drone_id in current_positions:
                local_pos = current_positions[drone_id]
                home = self._home_offsets.get(drone_id, (0, 0, 0))
                world_pos = (
                    local_pos[0] + home[0],  # World North
                    local_pos[1] + home[1],  # World East
                    local_pos[2] + home[2],  # World Down
                )
                world_positions[drone_id] = world_pos
            else:
                # Use home position if current position unknown
                world_positions[drone_id] = self._home_offsets.get(drone_id, (0, 0, 0))

        if formation_type == FormationType.LINE:
            # LINE: sort by East (smallest East gets westernmost slot 0)
            sorted_drones = sorted(drone_ids, key=lambda d: world_positions[d][1])
        else:
            # GRID: sort by row-major (North descending, then East ascending)
            # This matches how grid positions are generated
            sorted_drones = sorted(
                drone_ids,
                key=lambda d: (-world_positions[d][0], world_positions[d][1])
            )

        logger.debug(f"Formation slot assignment: {sorted_drones}")
        return sorted_drones

    def _fly_with_collision_avoidance(
        self,
        targets: dict[int, PositionNED],
        tolerance: float = 2.0,
        timeout: float = 30.0,
    ) -> bool:
        """Move drones to targets using 3-phase altitude separation.

        This method prevents collisions during formation transitions by:
        1. Phase 1: All drones climb to unique staggered altitudes
        2. Phase 2: All drones move horizontally at their staggered altitude
        3. Phase 3: All drones descend to final formation altitude

        The vertical separation ensures drones never occupy the same airspace
        during horizontal movement, even if paths would otherwise cross.

        Args:
            targets: Dict mapping drone_id to target (n, e, d) position
            tolerance: Distance threshold for "arrived" (meters)
            timeout: Maximum time for each phase (seconds)

        Returns:
            True if all drones reached their targets
        """
        if not targets:
            return True

        separation = self.config.collision_avoidance_separation
        drone_ids = list(targets.keys())
        num_drones = len(drone_ids)

        # Get current positions
        current_positions = self._get_all_positions(timeout=1.0)
        if not current_positions:
            logger.warning("Could not get positions, falling back to direct movement")
            return self._wait_for_positions(targets, tolerance, timeout)

        # Calculate staggered altitudes for each drone (based on sorted order)
        # Use drone's index in sorted list to assign altitude tier
        sorted_by_id = sorted(drone_ids)
        altitude_tiers = {
            drone_id: idx * separation
            for idx, drone_id in enumerate(sorted_by_id)
        }

        # Find the highest current altitude and highest target altitude
        current_alts = [-pos[2] for pos in current_positions.values() if pos]
        target_alts = [-pos[2] for pos in targets.values()]
        max_current_alt = max(current_alts) if current_alts else 10.0
        max_target_alt = max(target_alts) if target_alts else 10.0
        base_transition_alt = max(max_current_alt, max_target_alt) + separation

        print(f"  Collision avoidance: 3-phase transition (separation={separation}m)")

        # === PHASE 1: Climb to staggered altitudes ===
        print(f"  Phase 1/3: Climbing to staggered altitudes...")
        climb_targets = {}
        for drone_id in drone_ids:
            if drone_id in current_positions:
                current = current_positions[drone_id]
                stagger_alt = base_transition_alt + altitude_tiers[drone_id]
                # Keep current N/E, climb to staggered altitude
                climb_targets[drone_id] = (current[0], current[1], -stagger_alt)

        if climb_targets:
            phase1_ok = self._wait_for_positions(climb_targets, tolerance, timeout / 3)
            if not phase1_ok:
                logger.warning("Phase 1 (climb) incomplete, continuing anyway")

        # === PHASE 2: Move horizontally to target N/E ===
        print(f"  Phase 2/3: Moving horizontally to target positions...")
        horizontal_targets = {}
        for drone_id in drone_ids:
            target = targets[drone_id]
            stagger_alt = base_transition_alt + altitude_tiers[drone_id]
            # Move to target N/E, keep staggered altitude
            horizontal_targets[drone_id] = (target[0], target[1], -stagger_alt)

        if horizontal_targets:
            phase2_ok = self._wait_for_positions(horizontal_targets, tolerance, timeout / 3)
            if not phase2_ok:
                logger.warning("Phase 2 (horizontal) incomplete, continuing anyway")

        # === PHASE 3: Descend to final altitude ===
        print(f"  Phase 3/3: Descending to formation altitude...")
        phase3_ok = self._wait_for_positions(targets, tolerance, timeout / 3)

        if phase3_ok:
            logger.info("3-phase collision avoidance transition complete")
        else:
            logger.warning("Phase 3 (descend) incomplete")

        return phase3_ok

    def _calculate_home_offsets(self) -> dict[int, PositionNED]:
        """Calculate home position offsets for each drone based on spawn layout.

        Each drone's LOCAL_POSITION_NED is relative to its own home (spawn position).
        This method calculates the offset of each drone's home from drone 0's home,
        allowing us to transform world-frame formation positions to drone-local coords.

        Gazebo X maps to NED East, Gazebo Y maps to NED North.

        Returns:
            Dict mapping drone_id to (north, east, down) home offset from drone 0
        """
        import math

        offsets = {}
        layout = self.config.spawn_layout
        spacing = self.config.spawn_spacing
        num = self.config.num_drones

        if layout == "line":
            # Line along Gazebo X (East)
            for i in range(num):
                # Gazebo (i*spacing, 0, 0) -> NED (N=0, E=i*spacing, D=0)
                offsets[i] = (0.0, i * spacing, 0.0)
        else:
            # Grid layout
            cols = math.ceil(math.sqrt(num))
            for i in range(num):
                row, col = divmod(i, cols)
                # Gazebo (col*spacing, row*spacing, 0) -> NED (N=row*spacing, E=col*spacing, D=0)
                offsets[i] = (row * spacing, col * spacing, 0.0)

        return offsets

    def _world_to_local(
        self,
        world_pos: PositionNED,
        drone_id: int,
    ) -> PositionNED:
        """Convert world NED position to drone-local NED position.

        Args:
            world_pos: Position in world NED frame (relative to drone 0's home)
            drone_id: Target drone ID

        Returns:
            Position in drone's local NED frame
        """
        if drone_id not in self._home_offsets:
            return world_pos

        home = self._home_offsets[drone_id]
        return (
            world_pos[0] - home[0],  # North
            world_pos[1] - home[1],  # East
            world_pos[2] - home[2],  # Down
        )

    def _local_to_world(
        self,
        local_pos: PositionNED,
        drone_id: int,
    ) -> PositionNED:
        """Convert drone-local NED position to world NED position.

        Args:
            local_pos: Position in drone's local NED frame
            drone_id: Source drone ID

        Returns:
            Position in world NED frame (relative to drone 0's home)
        """
        if drone_id not in self._home_offsets:
            return local_pos

        home = self._home_offsets[drone_id]
        return (
            local_pos[0] + home[0],  # North
            local_pos[1] + home[1],  # East
            local_pos[2] + home[2],  # Down
        )

    def _get_all_positions_world(self, timeout: float = 0.5) -> dict[int, PositionNED]:
        """Get current positions from all drones in world frame.

        Unlike _get_all_positions() which returns local positions,
        this method converts them to world coordinates for comparison
        with formation calculations.

        Args:
            timeout: How long to wait per drone

        Returns:
            Dict mapping drone_id to world (north, east, down) position
        """
        local_positions = self._get_all_positions(timeout)
        world_positions = {}
        for drone_id, local_pos in local_positions.items():
            world_positions[drone_id] = self._local_to_world(local_pos, drone_id)
        return world_positions

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

    @property
    def num_drones(self) -> int:
        """Number of drones in the swarm."""
        return self.config.num_drones

    @property
    def drones(self) -> dict[int, mavutil.mavlink_connection]:
        """Dictionary of drone connections (drone_id -> connection)."""
        return self._connections

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
