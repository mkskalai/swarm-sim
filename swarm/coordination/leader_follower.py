"""Leader-follower swarm coordination.

Implements leader-follower mode where one drone leads and others maintain
fixed offsets from the leader's position. Includes automatic leader
promotion when the current leader fails.
"""

import time
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional, List, Callable, Tuple

logger = logging.getLogger(__name__)

# Position tuple: (north, east, down) in meters
PositionNED = Tuple[float, float, float]


class LeaderState(Enum):
    """Leader operational status."""
    ACTIVE = "active"
    FAILED = "failed"
    TRANSITIONING = "transitioning"


@dataclass
class FollowerOffset:
    """Offset from leader position.

    Attributes:
        north: Offset in NED North (meters)
        east: Offset in NED East (meters)
        down: Offset in NED Down (meters, negative = higher altitude)
    """
    north: float
    east: float
    down: float


@dataclass
class DroneStatus:
    """Tracked status of a drone in the swarm.

    Attributes:
        drone_id: Drone identifier
        is_leader: Whether this drone is the current leader
        last_heartbeat: Unix timestamp of last heartbeat
        position: Most recent known position
        offset: Offset from leader (None for leader itself)
    """
    drone_id: int
    is_leader: bool = False
    last_heartbeat: float = 0.0
    position: Optional[PositionNED] = None
    offset: Optional[FollowerOffset] = None


class LeaderFollowerController:
    """Manages leader-follower swarm behavior.

    One drone is designated leader. Followers maintain fixed offsets
    from the leader position. If the leader fails, the next drone in
    the priority list is automatically promoted.

    Example:
        lf = LeaderFollowerController(num_drones=6)
        lf.set_leader(0)

        # Set up V formation offsets
        positions = get_formation_positions(FormationType.V_FORMATION, 6)
        lf.set_offsets_from_positions(positions, leader_id=0)

        # In flight loop:
        leader_pos = get_drone_position(0)  # Get leader's current position
        targets = lf.get_all_target_positions(leader_pos)

        for drone_id, target in targets.items():
            send_position_command(drone_id, target)
    """

    def __init__(
        self,
        num_drones: int,
        heartbeat_timeout: float = 3.0,
    ):
        """Initialize leader-follower controller.

        Args:
            num_drones: Number of drones in the swarm
            heartbeat_timeout: Seconds without heartbeat before drone is considered failed
        """
        self.num_drones = num_drones
        self.heartbeat_timeout = heartbeat_timeout

        # Initialize drone statuses
        self.drones: dict[int, DroneStatus] = {
            i: DroneStatus(drone_id=i) for i in range(num_drones)
        }

        # Leader management
        self.leader_id: Optional[int] = None
        self.leader_priority: List[int] = list(range(num_drones))  # Default: 0, 1, 2, ...
        self.leader_state = LeaderState.ACTIVE

        # Callback for leader changes
        self._on_leader_change: Optional[Callable[[Optional[int], int], None]] = None

    def set_leader(self, drone_id: int) -> bool:
        """Designate a drone as the leader.

        Args:
            drone_id: ID of drone to make leader

        Returns:
            True if successful, False if invalid drone_id
        """
        if drone_id not in self.drones:
            logger.error(f"Invalid drone ID: {drone_id}")
            return False

        old_leader = self.leader_id

        # Clear old leader flag
        if self.leader_id is not None and self.leader_id in self.drones:
            self.drones[self.leader_id].is_leader = False
            self.drones[self.leader_id].offset = None

        # Set new leader
        self.leader_id = drone_id
        self.drones[drone_id].is_leader = True
        self.drones[drone_id].offset = None  # Leader has no offset
        self.leader_state = LeaderState.ACTIVE

        logger.info(f"Leader changed: {old_leader} -> {drone_id}")

        if self._on_leader_change and old_leader != drone_id:
            self._on_leader_change(old_leader, drone_id)

        return True

    def set_leader_priority(self, priority: List[int]) -> None:
        """Set the leader succession priority order.

        Args:
            priority: Ordered list of drone IDs for leader succession
        """
        self.leader_priority = priority

    def set_formation_offsets(
        self,
        offsets: List[Optional[FollowerOffset]],
        leader_id: Optional[int] = None
    ) -> None:
        """Set follower offsets relative to leader.

        Args:
            offsets: List of offsets, one per drone (None for leader)
            leader_id: Leader drone ID (uses current if None)
        """
        leader = leader_id if leader_id is not None else self.leader_id

        for i, offset in enumerate(offsets):
            if i == leader:
                self.drones[i].offset = None
            else:
                self.drones[i].offset = offset

    def set_offsets_from_positions(
        self,
        positions: List[PositionNED],
        leader_id: int = 0
    ) -> None:
        """Convert formation positions to leader-relative offsets.

        Given a list of absolute formation positions, calculates offsets
        relative to the leader's position.

        Args:
            positions: List of (north, east, down) formation positions
            leader_id: Index in positions list that corresponds to leader
        """
        if leader_id >= len(positions):
            logger.error(f"Leader ID {leader_id} out of range")
            return

        leader_pos = positions[leader_id]

        for i, pos in enumerate(positions):
            if i >= self.num_drones:
                break

            if i == leader_id:
                self.drones[i].offset = None
            else:
                offset = FollowerOffset(
                    north=pos[0] - leader_pos[0],
                    east=pos[1] - leader_pos[1],
                    down=pos[2] - leader_pos[2],
                )
                self.drones[i].offset = offset

    def update_drone_heartbeat(
        self,
        drone_id: int,
        position: Optional[PositionNED] = None
    ) -> None:
        """Update heartbeat timestamp and position for a drone.

        Call this regularly with telemetry data.

        Args:
            drone_id: Drone identifier
            position: Current (north, east, down) position
        """
        if drone_id in self.drones:
            self.drones[drone_id].last_heartbeat = time.time()
            if position is not None:
                self.drones[drone_id].position = position

    def check_leader_health(self) -> bool:
        """Check if leader is healthy, promote if needed.

        Returns:
            True if leader is healthy or promotion successful
        """
        if self.leader_id is None:
            return self._promote_next_leader()

        leader = self.drones[self.leader_id]
        elapsed = time.time() - leader.last_heartbeat

        if leader.last_heartbeat > 0 and elapsed > self.heartbeat_timeout:
            logger.warning(
                f"Leader {self.leader_id} heartbeat timeout "
                f"({elapsed:.1f}s > {self.heartbeat_timeout}s)"
            )
            self.leader_state = LeaderState.FAILED
            return self._promote_next_leader()

        return True

    def _promote_next_leader(self) -> bool:
        """Promote the next available drone to leader.

        Returns:
            True if promotion successful, False if no healthy drones
        """
        self.leader_state = LeaderState.TRANSITIONING

        for candidate_id in self.leader_priority:
            if candidate_id == self.leader_id:
                continue  # Skip failed leader

            if candidate_id not in self.drones:
                continue

            drone = self.drones[candidate_id]

            # Check if candidate is healthy
            if drone.last_heartbeat <= 0:
                continue

            elapsed = time.time() - drone.last_heartbeat
            if elapsed <= self.heartbeat_timeout:
                logger.info(f"Promoting drone {candidate_id} to leader")
                return self.set_leader(candidate_id)

        logger.error("No healthy drones available for leader promotion")
        return False

    def get_target_position(
        self,
        drone_id: int,
        leader_position: PositionNED
    ) -> Optional[PositionNED]:
        """Calculate target position for a drone.

        Args:
            drone_id: Drone identifier
            leader_position: Leader's current (north, east, down) position

        Returns:
            Target position for the drone, or None if it's the leader
        """
        if drone_id == self.leader_id:
            return None  # Leader doesn't follow anyone

        if drone_id not in self.drones:
            return None

        drone = self.drones[drone_id]
        if drone.offset is None:
            return None

        return (
            leader_position[0] + drone.offset.north,
            leader_position[1] + drone.offset.east,
            leader_position[2] + drone.offset.down,
        )

    def get_all_target_positions(
        self,
        leader_position: PositionNED
    ) -> dict[int, PositionNED]:
        """Get target positions for all followers.

        Args:
            leader_position: Leader's current (north, east, down) position

        Returns:
            Dict mapping drone_id -> target position (excludes leader)
        """
        targets = {}

        for drone_id in self.drones:
            pos = self.get_target_position(drone_id, leader_position)
            if pos is not None:
                targets[drone_id] = pos

        return targets

    def get_active_drones(self) -> List[int]:
        """Get list of healthy drone IDs.

        Returns:
            List of drone IDs with recent heartbeats
        """
        now = time.time()
        active = []

        for drone_id, drone in self.drones.items():
            if drone.last_heartbeat <= 0:
                continue
            if now - drone.last_heartbeat <= self.heartbeat_timeout:
                active.append(drone_id)

        return active

    def get_followers(self) -> List[int]:
        """Get list of follower drone IDs (excludes leader).

        Returns:
            List of drone IDs that are not the leader
        """
        return [
            drone_id for drone_id in self.drones
            if drone_id != self.leader_id
        ]

    def on_leader_change(
        self, callback: Callable[[Optional[int], int], None]
    ) -> None:
        """Register callback for leader changes.

        Args:
            callback: Function called when leader changes.
                     Receives (old_leader_id, new_leader_id).
                     old_leader_id may be None if this is initial leader.
        """
        self._on_leader_change = callback
