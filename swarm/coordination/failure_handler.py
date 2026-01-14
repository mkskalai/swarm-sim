"""Drone failure detection and graceful degradation.

Monitors drone health via heartbeat and position updates, detects failures,
and supports graceful degradation by removing failed drones from formations.
"""

import time
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Callable, Set, Tuple

logger = logging.getLogger(__name__)

# Position tuple: (north, east, down) in meters
PositionNED = Tuple[float, float, float]


class FailureType(Enum):
    """Types of drone failures."""
    HEARTBEAT_LOSS = "heartbeat_loss"
    POSITION_ERROR = "position_error"
    BATTERY_CRITICAL = "battery_critical"
    GEOFENCE_VIOLATION = "geofence_violation"
    COMMUNICATION_LOSS = "communication_loss"


@dataclass
class FailureEvent:
    """Record of a failure event.

    Attributes:
        drone_id: ID of the failed drone
        failure_type: Type of failure detected
        timestamp: Unix timestamp when failure was detected
        details: Human-readable description
        recovered: Whether the drone has recovered
    """
    drone_id: int
    failure_type: FailureType
    timestamp: float
    details: str = ""
    recovered: bool = False


@dataclass
class DroneHealth:
    """Health tracking for a single drone.

    Attributes:
        drone_id: Drone identifier
        last_heartbeat: Unix timestamp of last heartbeat
        last_position: Most recent known position (NED)
        last_position_time: When position was last updated
        battery_percent: Battery level (0-100)
        is_healthy: Whether drone is considered healthy
        failure_events: History of failure events
    """
    drone_id: int
    last_heartbeat: float = 0.0
    last_position: Optional[PositionNED] = None
    last_position_time: float = 0.0
    battery_percent: float = 100.0
    is_healthy: bool = True
    failure_events: List[FailureEvent] = field(default_factory=list)


class FailureHandler:
    """Monitors drone health and handles failures.

    Implements graceful degradation:
    1. Detect failure (heartbeat timeout, position drift, low battery)
    2. Mark drone as failed
    3. Remove from active formation
    4. Notify via callback
    5. Track recovery if drone comes back online

    Example:
        handler = FailureHandler(num_drones=6)
        handler.on_failure(lambda e: print(f"Drone {e.drone_id} failed!"))

        # In main loop:
        for drone_id in range(6):
            handler.update_heartbeat(drone_id, position=(n, e, d), battery=80.0)

        failures = handler.check_all_health()
        active = handler.get_active_drones()
    """

    def __init__(
        self,
        num_drones: int,
        heartbeat_timeout: float = 3.0,
        position_timeout: float = 2.0,
        position_error_threshold: float = 50.0,
        battery_critical_threshold: float = 15.0,
    ):
        """Initialize failure handler.

        Args:
            num_drones: Total number of drones to track
            heartbeat_timeout: Seconds without heartbeat before failure
            position_timeout: Seconds without position update before failure
            position_error_threshold: Max distance from expected position (meters)
            battery_critical_threshold: Battery percent below which is critical
        """
        self.num_drones = num_drones
        self.heartbeat_timeout = heartbeat_timeout
        self.position_timeout = position_timeout
        self.position_error_threshold = position_error_threshold
        self.battery_critical_threshold = battery_critical_threshold

        # Health tracking per drone
        self.health: dict[int, DroneHealth] = {
            i: DroneHealth(drone_id=i) for i in range(num_drones)
        }

        # Set of failed drone IDs
        self._failed_drones: Set[int] = set()

        # Callbacks
        self._on_failure: Optional[Callable[[FailureEvent], None]] = None
        self._on_recovery: Optional[Callable[[int], None]] = None

    def update_heartbeat(
        self,
        drone_id: int,
        position: Optional[PositionNED] = None,
        battery: Optional[float] = None
    ) -> None:
        """Update drone telemetry.

        Call this regularly (e.g., 4Hz) with latest drone telemetry.

        Args:
            drone_id: Drone identifier
            position: Current (north, east, down) position
            battery: Battery percentage (0-100)
        """
        if drone_id not in self.health:
            return

        h = self.health[drone_id]
        now = time.time()

        h.last_heartbeat = now

        if position is not None:
            h.last_position = position
            h.last_position_time = now

        if battery is not None:
            h.battery_percent = battery

        # Check if previously failed drone has recovered
        if drone_id in self._failed_drones:
            self._check_recovery(drone_id)

    def check_all_health(self) -> List[FailureEvent]:
        """Check health of all drones.

        Call this periodically (e.g., every second) to detect failures.

        Returns:
            List of new failure events detected this check
        """
        failures = []
        now = time.time()

        for drone_id, h in self.health.items():
            if drone_id in self._failed_drones:
                continue  # Already failed, skip

            # Check heartbeat timeout
            heartbeat_elapsed = now - h.last_heartbeat
            if h.last_heartbeat > 0 and heartbeat_elapsed > self.heartbeat_timeout:
                event = self._record_failure(
                    drone_id,
                    FailureType.HEARTBEAT_LOSS,
                    f"No heartbeat for {heartbeat_elapsed:.1f}s"
                )
                failures.append(event)
                continue

            # Check position update timeout
            position_elapsed = now - h.last_position_time
            if h.last_position_time > 0 and position_elapsed > self.position_timeout:
                event = self._record_failure(
                    drone_id,
                    FailureType.POSITION_ERROR,
                    f"No position update for {position_elapsed:.1f}s"
                )
                failures.append(event)
                continue

            # Check battery level
            if h.battery_percent < self.battery_critical_threshold:
                event = self._record_failure(
                    drone_id,
                    FailureType.BATTERY_CRITICAL,
                    f"Battery at {h.battery_percent:.1f}%"
                )
                failures.append(event)
                continue

        return failures

    def check_position_error(
        self,
        drone_id: int,
        actual: PositionNED,
        expected: PositionNED
    ) -> Optional[FailureEvent]:
        """Check if drone position deviates too much from expected.

        Args:
            drone_id: Drone identifier
            actual: Actual (north, east, down) position
            expected: Expected (north, east, down) position

        Returns:
            FailureEvent if position error exceeds threshold, None otherwise
        """
        if drone_id in self._failed_drones:
            return None

        dist = (
            (actual[0] - expected[0]) ** 2 +
            (actual[1] - expected[1]) ** 2 +
            (actual[2] - expected[2]) ** 2
        ) ** 0.5

        if dist > self.position_error_threshold:
            return self._record_failure(
                drone_id,
                FailureType.POSITION_ERROR,
                f"Position error: {dist:.1f}m (threshold: {self.position_error_threshold}m)"
            )

        return None

    def _record_failure(
        self,
        drone_id: int,
        failure_type: FailureType,
        details: str
    ) -> FailureEvent:
        """Record a failure event and update state."""
        event = FailureEvent(
            drone_id=drone_id,
            failure_type=failure_type,
            timestamp=time.time(),
            details=details,
        )

        self.health[drone_id].is_healthy = False
        self.health[drone_id].failure_events.append(event)
        self._failed_drones.add(drone_id)

        logger.warning(f"FAILURE: Drone {drone_id} - {failure_type.value}: {details}")

        if self._on_failure:
            self._on_failure(event)

        return event

    def _check_recovery(self, drone_id: int) -> None:
        """Check if a failed drone has recovered."""
        h = self.health[drone_id]
        now = time.time()

        # Check all health criteria (with hysteresis for battery)
        heartbeat_ok = (now - h.last_heartbeat) <= self.heartbeat_timeout
        position_ok = (now - h.last_position_time) <= self.position_timeout
        battery_ok = h.battery_percent >= (self.battery_critical_threshold + 5)  # Hysteresis

        if heartbeat_ok and position_ok and battery_ok:
            logger.info(f"RECOVERY: Drone {drone_id} has recovered")

            h.is_healthy = True
            self._failed_drones.discard(drone_id)

            # Mark last failure as recovered
            for event in reversed(h.failure_events):
                if not event.recovered:
                    event.recovered = True
                    break

            if self._on_recovery:
                self._on_recovery(drone_id)

    def get_active_drones(self) -> List[int]:
        """Get list of healthy drone IDs.

        Returns:
            List of drone IDs that are not failed
        """
        return [
            drone_id for drone_id in range(self.num_drones)
            if drone_id not in self._failed_drones
        ]

    def get_failed_drones(self) -> List[int]:
        """Get list of failed drone IDs.

        Returns:
            List of drone IDs that have failed
        """
        return list(self._failed_drones)

    def manually_mark_failed(self, drone_id: int, reason: str = "") -> None:
        """Manually mark a drone as failed.

        Args:
            drone_id: Drone to mark as failed
            reason: Reason for manual failure marking
        """
        if drone_id in self._failed_drones:
            return

        self._record_failure(
            drone_id,
            FailureType.COMMUNICATION_LOSS,
            f"Manually marked failed: {reason}" if reason else "Manually marked failed"
        )

    def manually_mark_recovered(self, drone_id: int) -> None:
        """Manually mark a drone as recovered.

        Args:
            drone_id: Drone to mark as recovered
        """
        if drone_id not in self._failed_drones:
            return

        h = self.health[drone_id]
        h.is_healthy = True
        h.last_heartbeat = time.time()
        h.last_position_time = time.time()
        self._failed_drones.discard(drone_id)

        for event in reversed(h.failure_events):
            if not event.recovered:
                event.recovered = True
                break

        logger.info(f"RECOVERY: Drone {drone_id} manually marked recovered")

        if self._on_recovery:
            self._on_recovery(drone_id)

    def on_failure(self, callback: Callable[[FailureEvent], None]) -> None:
        """Register failure callback.

        Args:
            callback: Function called when a drone fails
        """
        self._on_failure = callback

    def on_recovery(self, callback: Callable[[int], None]) -> None:
        """Register recovery callback.

        Args:
            callback: Function called when a drone recovers (receives drone_id)
        """
        self._on_recovery = callback

    def get_failure_history(self, drone_id: int) -> List[FailureEvent]:
        """Get failure history for a drone.

        Args:
            drone_id: Drone identifier

        Returns:
            List of failure events for the drone
        """
        if drone_id in self.health:
            return self.health[drone_id].failure_events
        return []

    @staticmethod
    def redistribute_formation(
        original_positions: List[PositionNED],
        active_drone_ids: List[int],
    ) -> dict[int, PositionNED]:
        """Redistribute formation positions among active drones.

        Simple strategy: each active drone keeps its original position.
        Failed drone positions are skipped.

        Args:
            original_positions: Original positions for all drones
            active_drone_ids: IDs of drones still active

        Returns:
            Dict mapping drone_id -> new position
        """
        new_positions = {}

        for drone_id in active_drone_ids:
            if drone_id < len(original_positions):
                new_positions[drone_id] = original_positions[drone_id]

        return new_positions
