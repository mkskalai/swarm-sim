"""Single drone controller using MAVSDK."""

import asyncio
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable, AsyncIterator

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
from mavsdk.telemetry import LandedState, FlightMode

from .config import DroneConfig

logger = logging.getLogger(__name__)


class DroneState(Enum):
    """Drone operational state."""
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    IN_FLIGHT = "in_flight"
    LANDING = "landing"
    LANDED = "landed"


@dataclass
class Position:
    """3D position in local NED frame."""
    north: float  # meters
    east: float   # meters
    down: float   # meters (negative = up)
    yaw: float = 0.0  # degrees

    @property
    def altitude(self) -> float:
        """Altitude above ground (positive up)."""
        return -self.down


@dataclass
class GlobalPosition:
    """GPS position."""
    latitude: float   # degrees
    longitude: float  # degrees
    altitude: float   # meters AMSL
    yaw: float = 0.0  # degrees


class Drone:
    """Controller for a single drone via MAVSDK.

    Provides async interface for:
    - Connection management
    - Basic flight commands (arm, takeoff, land, goto)
    - Offboard control mode
    - Telemetry subscriptions
    """

    def __init__(self, config: Optional[DroneConfig] = None):
        """Initialize drone controller.

        Args:
            config: Drone configuration. Uses defaults if not provided.
        """
        self.config = config or DroneConfig()
        self._system = System()
        self._state = DroneState.DISCONNECTED
        self._is_offboard = False
        self._telemetry_tasks: list[asyncio.Task] = []

        # Cached telemetry
        self._position: Optional[Position] = None
        self._global_position: Optional[GlobalPosition] = None
        self._battery_percent: float = 0.0
        self._flight_mode: Optional[FlightMode] = None
        self._landed_state: Optional[LandedState] = None

    @property
    def state(self) -> DroneState:
        """Current drone state."""
        return self._state

    @property
    def position(self) -> Optional[Position]:
        """Current local position (NED frame)."""
        return self._position

    @property
    def global_position(self) -> Optional[GlobalPosition]:
        """Current GPS position."""
        return self._global_position

    @property
    def battery_percent(self) -> float:
        """Battery level (0-100)."""
        return self._battery_percent

    @property
    def is_connected(self) -> bool:
        """Whether drone is connected."""
        return self._state != DroneState.DISCONNECTED

    @property
    def is_armed(self) -> bool:
        """Whether drone is armed."""
        return self._state in (
            DroneState.ARMED,
            DroneState.TAKING_OFF,
            DroneState.IN_FLIGHT,
            DroneState.LANDING,
        )

    async def connect(self, timeout: Optional[float] = None) -> bool:
        """Connect to the drone.

        Args:
            timeout: Connection timeout in seconds. Uses config default if None.

        Returns:
            True if connection successful.
        """
        timeout = timeout or self.config.connection_timeout

        logger.info(f"Connecting to drone at {self.config.system_address}...")

        await self._system.connect(system_address=self.config.system_address)

        # Wait for connection with timeout
        try:
            async with asyncio.timeout(timeout):
                async for state in self._system.core.connection_state():
                    if state.is_connected:
                        logger.info("Drone connected!")
                        self._state = DroneState.CONNECTED
                        break
        except TimeoutError:
            logger.error(f"Connection timeout after {timeout}s")
            return False

        # Start telemetry subscriptions
        await self._start_telemetry()

        # Wait for GPS fix
        logger.info("Waiting for GPS fix...")
        try:
            async with asyncio.timeout(timeout):
                async for health in self._system.telemetry.health():
                    if health.is_global_position_ok and health.is_home_position_ok:
                        logger.info("GPS fix acquired")
                        break
        except TimeoutError:
            logger.warning("GPS fix timeout - continuing anyway (may be SITL)")

        return True

    async def disconnect(self) -> None:
        """Disconnect from the drone."""
        # Cancel telemetry tasks
        for task in self._telemetry_tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self._telemetry_tasks.clear()

        self._state = DroneState.DISCONNECTED
        logger.info("Disconnected from drone")

    async def arm(self) -> bool:
        """Arm the drone.

        Returns:
            True if arming successful.
        """
        if not self.is_connected:
            logger.error("Cannot arm: not connected")
            return False

        logger.info("Arming drone...")
        try:
            await self._system.action.arm()
            self._state = DroneState.ARMED
            logger.info("Drone armed")
            return True
        except Exception as e:
            logger.error(f"Arming failed: {e}")
            return False

    async def disarm(self) -> bool:
        """Disarm the drone.

        Returns:
            True if disarming successful.
        """
        logger.info("Disarming drone...")
        try:
            await self._system.action.disarm()
            self._state = DroneState.CONNECTED
            logger.info("Drone disarmed")
            return True
        except Exception as e:
            logger.error(f"Disarming failed: {e}")
            return False

    async def takeoff(self, altitude: Optional[float] = None) -> bool:
        """Take off to specified altitude.

        Args:
            altitude: Target altitude in meters. Uses config default if None.

        Returns:
            True if takeoff command accepted.
        """
        altitude = altitude or self.config.default_altitude

        if not self.is_armed:
            logger.info("Not armed, arming first...")
            if not await self.arm():
                return False

        # Wait for GPS fix before takeoff (required for GUIDED mode)
        gps_timeout = 30.0  # seconds
        gps_start = asyncio.get_event_loop().time()
        gps_ok = False

        try:
            async for health in self._system.telemetry.health():
                if health.is_global_position_ok:
                    logger.info("GPS position OK, proceeding with takeoff")
                    gps_ok = True
                    break

                elapsed = asyncio.get_event_loop().time() - gps_start
                if elapsed > gps_timeout:
                    logger.error(f"GPS timeout after {gps_timeout}s - health status: "
                                f"gps={health.is_gyrometer_calibration_ok}, "
                                f"accel={health.is_accelerometer_calibration_ok}, "
                                f"mag={health.is_magnetometer_calibration_ok}, "
                                f"local_pos={health.is_local_position_ok}, "
                                f"global_pos={health.is_global_position_ok}, "
                                f"home_pos={health.is_home_position_ok}")
                    break

                logger.info(f"Waiting for GPS... (elapsed: {elapsed:.1f}s)")
                await asyncio.sleep(2)
        except Exception as e:
            logger.warning(f"Could not check GPS health: {e}")

        if not gps_ok:
            logger.error("GPS not available, cannot takeoff in GUIDED mode")
            return False

        logger.info(f"Taking off to {altitude}m...")
        self._state = DroneState.TAKING_OFF

        try:
            await self._system.action.set_takeoff_altitude(altitude)
            await self._system.action.takeoff()

            # Wait for takeoff to complete
            async for landed_state in self._system.telemetry.landed_state():
                if landed_state == LandedState.IN_AIR:
                    self._state = DroneState.IN_FLIGHT
                    logger.info("Takeoff complete")
                    break

            return True
        except Exception as e:
            logger.error(f"Takeoff failed: {e}")
            return False

    async def land(self) -> bool:
        """Land the drone.

        Returns:
            True if landing successful.
        """
        logger.info("Landing...")
        self._state = DroneState.LANDING

        try:
            # Exit offboard mode if active
            if self._is_offboard:
                await self.stop_offboard()

            await self._system.action.land()

            # Wait for landing
            async for landed_state in self._system.telemetry.landed_state():
                if landed_state == LandedState.ON_GROUND:
                    self._state = DroneState.LANDED
                    logger.info("Landing complete")
                    break

            return True
        except Exception as e:
            logger.error(f"Landing failed: {e}")
            return False

    async def return_to_launch(self) -> bool:
        """Return to launch position.

        Returns:
            True if RTL command accepted.
        """
        logger.info("Returning to launch...")
        try:
            await self._system.action.return_to_launch()
            return True
        except Exception as e:
            logger.error(f"RTL failed: {e}")
            return False

    async def goto_location(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        yaw: float = 0.0,
    ) -> bool:
        """Go to GPS location.

        Args:
            latitude: Target latitude in degrees.
            longitude: Target longitude in degrees.
            altitude: Target altitude in meters AMSL.
            yaw: Target heading in degrees.

        Returns:
            True if goto command accepted.
        """
        logger.info(f"Going to ({latitude}, {longitude}, {altitude}m)...")
        try:
            await self._system.action.goto_location(latitude, longitude, altitude, yaw)
            return True
        except Exception as e:
            logger.error(f"Goto location failed: {e}")
            return False

    async def start_offboard(self) -> bool:
        """Start offboard control mode.

        Must set initial setpoint before calling this.

        Returns:
            True if offboard mode started.
        """
        if self._is_offboard:
            return True

        logger.info("Starting offboard mode...")
        try:
            # Set initial setpoint (hold current position)
            await self._system.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            await self._system.offboard.start()
            self._is_offboard = True
            logger.info("Offboard mode active")
            return True
        except OffboardError as e:
            logger.error(f"Failed to start offboard: {e}")
            return False

    async def stop_offboard(self) -> bool:
        """Stop offboard control mode.

        Returns:
            True if offboard mode stopped.
        """
        if not self._is_offboard:
            return True

        logger.info("Stopping offboard mode...")
        try:
            await self._system.offboard.stop()
            self._is_offboard = False
            logger.info("Offboard mode stopped")
            return True
        except OffboardError as e:
            logger.error(f"Failed to stop offboard: {e}")
            return False

    async def set_position_ned(
        self,
        north: float,
        east: float,
        down: float,
        yaw: float = 0.0,
    ) -> None:
        """Set position setpoint in NED frame (offboard mode).

        Args:
            north: North position in meters.
            east: East position in meters.
            down: Down position in meters (negative = up).
            yaw: Heading in degrees.
        """
        await self._system.offboard.set_position_ned(
            PositionNedYaw(north, east, down, yaw)
        )

    async def set_velocity_ned(
        self,
        north: float,
        east: float,
        down: float,
        yaw: float = 0.0,
    ) -> None:
        """Set velocity setpoint in NED frame (offboard mode).

        Args:
            north: North velocity in m/s.
            east: East velocity in m/s.
            down: Down velocity in m/s (negative = up).
            yaw: Heading in degrees.
        """
        await self._system.offboard.set_velocity_ned(
            VelocityNedYaw(north, east, down, yaw)
        )

    async def goto_position_ned(
        self,
        north: float,
        east: float,
        down: float,
        yaw: float = 0.0,
        tolerance: float = 0.5,
    ) -> bool:
        """Go to position in NED frame and wait for arrival (offboard mode).

        Args:
            north: North position in meters.
            east: East position in meters.
            down: Down position in meters (negative = up).
            yaw: Heading in degrees.
            tolerance: Arrival tolerance in meters.

        Returns:
            True if position reached.
        """
        if not self._is_offboard:
            if not await self.start_offboard():
                return False

        target = Position(north, east, down, yaw)
        logger.info(f"Going to NED position ({north}, {east}, {down})")

        while True:
            await self.set_position_ned(north, east, down, yaw)
            await asyncio.sleep(1.0 / self.config.position_update_rate)

            if self._position:
                dist = (
                    (self._position.north - target.north) ** 2
                    + (self._position.east - target.east) ** 2
                    + (self._position.down - target.down) ** 2
                ) ** 0.5

                if dist < tolerance:
                    logger.info("Position reached")
                    return True

    async def _start_telemetry(self) -> None:
        """Start background telemetry subscriptions."""

        async def position_task():
            async for pos in self._system.telemetry.position_velocity_ned():
                self._position = Position(
                    north=pos.position.north_m,
                    east=pos.position.east_m,
                    down=pos.position.down_m,
                )

        async def global_position_task():
            async for pos in self._system.telemetry.position():
                self._global_position = GlobalPosition(
                    latitude=pos.latitude_deg,
                    longitude=pos.longitude_deg,
                    altitude=pos.absolute_altitude_m,
                )

        async def battery_task():
            async for battery in self._system.telemetry.battery():
                self._battery_percent = battery.remaining_percent * 100

        async def flight_mode_task():
            async for mode in self._system.telemetry.flight_mode():
                self._flight_mode = mode

        async def landed_state_task():
            async for state in self._system.telemetry.landed_state():
                self._landed_state = state

        self._telemetry_tasks = [
            asyncio.create_task(position_task()),
            asyncio.create_task(global_position_task()),
            asyncio.create_task(battery_task()),
            asyncio.create_task(flight_mode_task()),
            asyncio.create_task(landed_state_task()),
        ]

    def __repr__(self) -> str:
        return f"Drone(instance={self.config.instance_id}, state={self._state.value})"
