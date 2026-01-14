"""Fleet manager for coordinating multiple drones.

This module provides the Fleet class for managing multiple drone instances
with concurrent operations for efficient swarm control.

Example:
    from swarm.core import Fleet, FleetConfig

    config = FleetConfig(num_drones=3)
    fleet = Fleet(config)

    async def main():
        await fleet.connect_all()
        await fleet.arm_all()
        await fleet.takeoff_all(altitude=10.0)
        # ... fly around ...
        await fleet.land_all()
        await fleet.disconnect_all()
"""

import asyncio
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Iterator

from .drone import Drone, DroneState, Position
from .config import DroneConfig, FleetConfig

logger = logging.getLogger(__name__)


class FleetState(Enum):
    """Fleet operational state."""

    UNINITIALIZED = "uninitialized"
    CONNECTING = "connecting"
    READY = "ready"
    ARMED = "armed"
    IN_FLIGHT = "in_flight"
    LANDING = "landing"
    ERROR = "error"


@dataclass
class FleetStatus:
    """Status summary of the fleet."""

    total: int
    connected: int
    armed: int
    in_flight: int
    errors: list[str] = field(default_factory=list)

    @property
    def all_connected(self) -> bool:
        """Check if all drones are connected."""
        return self.connected == self.total

    @property
    def all_armed(self) -> bool:
        """Check if all drones are armed."""
        return self.armed == self.total

    @property
    def all_in_flight(self) -> bool:
        """Check if all drones are in flight."""
        return self.in_flight == self.total

    @property
    def has_errors(self) -> bool:
        """Check if any errors occurred."""
        return len(self.errors) > 0


class Fleet:
    """Manager for coordinating multiple drones.

    Provides concurrent operations across all drones in the fleet:
    - Parallel connection establishment
    - Synchronized arming and takeoff
    - Formation flying commands
    - Graceful error handling for individual drone failures

    The Fleet class wraps multiple Drone instances and provides
    batch operations that execute concurrently using asyncio.gather().

    Attributes:
        config: Fleet configuration.
        drones: List of Drone instances.
        state: Current fleet operational state.
    """

    def __init__(self, config: Optional[FleetConfig] = None):
        """Initialize fleet manager.

        Args:
            config: Fleet configuration. Uses defaults if not provided.
        """
        self.config = config or FleetConfig()
        self._drones: list[Drone] = []
        self._state = FleetState.UNINITIALIZED

        # Create drone instances
        logger.info(f"Creating fleet with {self.config.num_drones} drones")
        for i in range(self.config.num_drones):
            drone_config = self.config.get_drone_config(i)
            logger.debug(f"Drone {i}: {drone_config.system_address}")
            self._drones.append(Drone(drone_config))

    @property
    def drones(self) -> list[Drone]:
        """List of drones in the fleet."""
        return self._drones

    @property
    def state(self) -> FleetState:
        """Current fleet state."""
        return self._state

    @property
    def num_drones(self) -> int:
        """Number of drones in the fleet."""
        return len(self._drones)

    def get_status(self) -> FleetStatus:
        """Get current fleet status summary.

        Returns:
            FleetStatus with counts of connected, armed, in-flight drones.
        """
        connected = sum(1 for d in self._drones if d.is_connected)
        armed = sum(1 for d in self._drones if d.is_armed)
        in_flight = sum(
            1 for d in self._drones if d.state == DroneState.IN_FLIGHT
        )
        errors = [
            f"Drone {i}: {d.state.value}"
            for i, d in enumerate(self._drones)
            if d.state == DroneState.DISCONNECTED and self._state != FleetState.UNINITIALIZED
        ]

        return FleetStatus(
            total=len(self._drones),
            connected=connected,
            armed=armed,
            in_flight=in_flight,
            errors=errors,
        )

    async def connect_all(self, timeout: float = 30.0) -> bool:
        """Connect to all drones concurrently.

        Args:
            timeout: Connection timeout per drone in seconds.

        Returns:
            True if all drones connected successfully.
        """
        self._state = FleetState.CONNECTING
        logger.info(f"Connecting to {len(self._drones)} drones...")

        # Connect concurrently
        results = await asyncio.gather(
            *[drone.connect(timeout) for drone in self._drones],
            return_exceptions=True,
        )

        # Check results
        success_count = 0
        for i, result in enumerate(results):
            if result is True:
                success_count += 1
                logger.info(f"Drone {i}: Connected")
            elif isinstance(result, Exception):
                logger.error(f"Drone {i}: Connection failed - {result}")
            else:
                logger.error(f"Drone {i}: Connection failed")

        success = success_count == len(self._drones)

        if success:
            self._state = FleetState.READY
            logger.info(f"All {len(self._drones)} drones connected!")
        else:
            self._state = FleetState.ERROR
            logger.error(
                f"Fleet connection incomplete: {success_count}/{len(self._drones)} connected"
            )

        return success

    async def disconnect_all(self) -> None:
        """Disconnect from all drones."""
        logger.info("Disconnecting from all drones...")

        await asyncio.gather(
            *[drone.disconnect() for drone in self._drones],
            return_exceptions=True,
        )

        self._state = FleetState.UNINITIALIZED
        logger.info("All drones disconnected")

    async def arm_all(self) -> bool:
        """Arm all drones concurrently.

        Returns:
            True if all drones armed successfully.
        """
        logger.info("Arming all drones...")

        results = await asyncio.gather(
            *[drone.arm() for drone in self._drones],
            return_exceptions=True,
        )

        success_count = 0
        for i, result in enumerate(results):
            if result is True:
                success_count += 1
                logger.info(f"Drone {i}: Armed")
            elif isinstance(result, Exception):
                logger.error(f"Drone {i}: Arm failed - {result}")
            else:
                logger.error(f"Drone {i}: Arm failed")

        success = success_count == len(self._drones)

        if success:
            self._state = FleetState.ARMED
            logger.info("All drones armed!")
        else:
            logger.error(
                f"Fleet arming incomplete: {success_count}/{len(self._drones)} armed"
            )

        return success

    async def disarm_all(self) -> bool:
        """Disarm all drones concurrently.

        Returns:
            True if all drones disarmed successfully.
        """
        logger.info("Disarming all drones...")

        results = await asyncio.gather(
            *[drone.disarm() for drone in self._drones],
            return_exceptions=True,
        )

        success = all(r is True for r in results if not isinstance(r, Exception))

        if success:
            self._state = FleetState.READY
            logger.info("All drones disarmed")

        return success

    async def takeoff_all(self, altitude: float = 10.0) -> bool:
        """Take off all drones concurrently.

        Args:
            altitude: Target altitude in meters.

        Returns:
            True if all drones took off successfully.
        """
        logger.info(f"Taking off all drones to {altitude}m...")

        results = await asyncio.gather(
            *[drone.takeoff(altitude) for drone in self._drones],
            return_exceptions=True,
        )

        success_count = 0
        for i, result in enumerate(results):
            if result is True:
                success_count += 1
                logger.info(f"Drone {i}: Takeoff complete")
            elif isinstance(result, Exception):
                logger.error(f"Drone {i}: Takeoff failed - {result}")
            else:
                logger.error(f"Drone {i}: Takeoff failed")

        success = success_count == len(self._drones)

        if success:
            self._state = FleetState.IN_FLIGHT
            logger.info("All drones in flight!")
        else:
            logger.error(
                f"Fleet takeoff incomplete: {success_count}/{len(self._drones)} in flight"
            )

        return success

    async def land_all(self) -> bool:
        """Land all drones concurrently.

        Returns:
            True if all drones landed successfully.
        """
        logger.info("Landing all drones...")
        self._state = FleetState.LANDING

        results = await asyncio.gather(
            *[drone.land() for drone in self._drones],
            return_exceptions=True,
        )

        success_count = 0
        for i, result in enumerate(results):
            if result is True:
                success_count += 1
                logger.info(f"Drone {i}: Landed")
            elif isinstance(result, Exception):
                logger.error(f"Drone {i}: Landing failed - {result}")
            else:
                logger.error(f"Drone {i}: Landing failed")

        success = success_count == len(self._drones)

        if success:
            self._state = FleetState.READY
            logger.info("All drones landed!")
        else:
            logger.error(
                f"Fleet landing incomplete: {success_count}/{len(self._drones)} landed"
            )

        return success

    async def return_to_launch_all(self) -> bool:
        """Command all drones to return to launch position.

        Returns:
            True if all drones acknowledged RTL command.
        """
        logger.info("Commanding all drones to return to launch...")

        results = await asyncio.gather(
            *[drone.return_to_launch() for drone in self._drones],
            return_exceptions=True,
        )

        success = all(r is True for r in results if not isinstance(r, Exception))

        if success:
            self._state = FleetState.LANDING
            logger.info("All drones returning to launch")

        return success

    async def start_offboard_all(self) -> bool:
        """Start offboard mode on all drones.

        Returns:
            True if all drones entered offboard mode.
        """
        logger.info("Starting offboard mode on all drones...")

        results = await asyncio.gather(
            *[drone.start_offboard() for drone in self._drones],
            return_exceptions=True,
        )

        success = all(r is True for r in results if not isinstance(r, Exception))

        if success:
            logger.info("All drones in offboard mode")
        else:
            logger.error("Failed to start offboard mode on some drones")

        return success

    async def stop_offboard_all(self) -> bool:
        """Stop offboard mode on all drones.

        Returns:
            True if all drones exited offboard mode.
        """
        logger.info("Stopping offboard mode on all drones...")

        results = await asyncio.gather(
            *[drone.stop_offboard() for drone in self._drones],
            return_exceptions=True,
        )

        success = all(r is True for r in results if not isinstance(r, Exception))

        if success:
            logger.info("All drones exited offboard mode")

        return success

    async def goto_positions(
        self,
        positions: list[Position],
        tolerance: float = 0.5,
    ) -> bool:
        """Move all drones to specified positions concurrently.

        Each drone moves to its corresponding position in the list.

        Args:
            positions: Target positions for each drone (NED frame).
                       Must have same length as number of drones.
            tolerance: Arrival tolerance in meters.

        Returns:
            True if all drones reached their positions.

        Raises:
            ValueError: If positions list length doesn't match drone count.
        """
        if len(positions) != len(self._drones):
            raise ValueError(
                f"Expected {len(self._drones)} positions, got {len(positions)}"
            )

        logger.info(f"Moving {len(self._drones)} drones to positions...")

        # Ensure offboard mode is active
        await self.start_offboard_all()

        # Move to positions
        results = await asyncio.gather(
            *[
                drone.goto_position_ned(
                    pos.north,
                    pos.east,
                    pos.down,
                    pos.yaw,
                    tolerance,
                )
                for drone, pos in zip(self._drones, positions)
            ],
            return_exceptions=True,
        )

        success_count = 0
        for i, result in enumerate(results):
            if result is True:
                success_count += 1
                logger.debug(f"Drone {i}: Reached position")
            elif isinstance(result, Exception):
                logger.error(f"Drone {i}: Position error - {result}")

        success = success_count == len(self._drones)

        if success:
            logger.info("All drones reached their positions")
        else:
            logger.error(
                f"Position command incomplete: {success_count}/{len(self._drones)}"
            )

        return success

    async def set_positions(self, positions: list[Position]) -> bool:
        """Set position setpoints for all drones (non-blocking).

        Unlike goto_positions(), this method sets the setpoints once
        without waiting for the drones to arrive.

        Args:
            positions: Target positions for each drone (NED frame).

        Returns:
            True if all setpoints were set successfully.

        Raises:
            ValueError: If positions list length doesn't match drone count.
        """
        if len(positions) != len(self._drones):
            raise ValueError(
                f"Expected {len(self._drones)} positions, got {len(positions)}"
            )

        results = await asyncio.gather(
            *[
                drone.set_position_ned(
                    pos.north,
                    pos.east,
                    pos.down,
                    pos.yaw,
                )
                for drone, pos in zip(self._drones, positions)
            ],
            return_exceptions=True,
        )

        return all(r is True for r in results if not isinstance(r, Exception))

    def get_positions(self) -> list[Optional[Position]]:
        """Get current positions of all drones.

        Returns:
            List of Position objects (or None if not available).
        """
        return [drone.position for drone in self._drones]

    def get_battery_levels(self) -> list[float]:
        """Get battery levels of all drones.

        Returns:
            List of battery percentages (0-100).
        """
        return [drone.battery_percent for drone in self._drones]

    # Container protocol support

    def __len__(self) -> int:
        """Return number of drones in fleet."""
        return len(self._drones)

    def __getitem__(self, index: int) -> Drone:
        """Get drone by index."""
        return self._drones[index]

    def __iter__(self) -> Iterator[Drone]:
        """Iterate over drones in fleet."""
        return iter(self._drones)

    def __repr__(self) -> str:
        """String representation."""
        return f"Fleet(num_drones={len(self._drones)}, state={self._state.value})"
