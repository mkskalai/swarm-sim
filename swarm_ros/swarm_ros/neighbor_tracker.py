"""
NeighborTracker - P2P neighbor discovery and tracking.

Tracks the state of all peer drones in the swarm via ROS2 topics.
Provides collision prediction and neighbor health monitoring.

Example:
    tracker = NeighborTracker(node, own_drone_id=0, num_drones=3)
    tracker.start()  # Starts subscribing to peer topics

    # Later, in control loop:
    neighbors = tracker.get_all_neighbors()
    collision = tracker.predict_collision(own_position, own_velocity)
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from rclpy.node import Node

logger = logging.getLogger(__name__)


@dataclass
class NeighborState:
    """State of a neighbor drone."""

    drone_id: int
    position_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0
    state: int = 0  # DroneState enum value
    battery_percent: int = 100
    is_healthy: bool = True
    last_update_time: float = 0.0
    is_stale: bool = False


@dataclass
class CollisionWarning:
    """Predicted collision with a neighbor."""

    other_drone_id: int
    time_to_collision: float  # seconds
    min_distance: float  # meters
    avoidance_vector: tuple[float, float, float]  # suggested NED velocity change


@dataclass
class NeighborTrackerConfig:
    """Configuration for NeighborTracker."""

    # Timeout before marking neighbor as stale (seconds)
    stale_timeout: float = 3.0

    # Timeout before marking neighbor as failed (seconds)
    failed_timeout: float = 10.0

    # Collision prediction time horizon (seconds)
    collision_horizon: float = 5.0

    # Minimum safe distance (meters)
    safe_distance: float = 3.0

    # Position update rate expected from peers (Hz)
    expected_update_rate: float = 10.0


class NeighborTracker:
    """
    Tracks the state of peer drones via ROS2 topic subscriptions.

    Features:
    - Subscribes to /drone_N/state topics for all peers
    - Detects stale/failed neighbors based on timeout
    - Predicts collisions based on position/velocity extrapolation
    - Provides callbacks for neighbor events (join, leave, collision)
    """

    def __init__(
        self,
        node: Node,
        own_drone_id: int,
        num_drones: int,
        config: NeighborTrackerConfig | None = None,
    ):
        """
        Initialize NeighborTracker.

        Args:
            node: ROS2 node to create subscriptions on
            own_drone_id: This drone's ID (won't subscribe to self)
            num_drones: Total number of drones in swarm
            config: Optional configuration
        """
        self.node = node
        self.own_drone_id = own_drone_id
        self.num_drones = num_drones
        self.config = config or NeighborTrackerConfig()

        # Neighbor state storage
        self._neighbors: dict[int, NeighborState] = {}

        # Callbacks
        self._on_neighbor_joined: list[Callable[[int], None]] = []
        self._on_neighbor_lost: list[Callable[[int], None]] = []
        self._on_collision_warning: list[Callable[[CollisionWarning], None]] = []

        # Track which neighbors we've seen
        self._known_neighbors: set[int] = set()

        # Subscriptions (created in start())
        self._subscriptions = []

        self._started = False

    def start(self) -> None:
        """Start tracking neighbors by creating subscriptions."""
        if self._started:
            logger.warning("NeighborTracker already started")
            return

        # Import here to avoid circular imports and allow testing without ROS2
        try:
            from swarm_ros.msg import DroneState
        except ImportError:
            logger.error("swarm_ros messages not built. Run 'colcon build' first.")
            raise

        # Subscribe to all peer drone state topics
        for drone_id in range(self.num_drones):
            if drone_id == self.own_drone_id:
                continue

            topic = f"/drone_{drone_id}/state"
            sub = self.node.create_subscription(
                DroneState,
                topic,
                lambda msg, did=drone_id: self._on_drone_state(did, msg),
                10,  # QoS depth
            )
            self._subscriptions.append(sub)
            logger.info(f"Subscribed to {topic}")

            # Initialize neighbor state
            self._neighbors[drone_id] = NeighborState(drone_id=drone_id)

        self._started = True
        logger.info(
            f"NeighborTracker started for drone {self.own_drone_id}, "
            f"tracking {self.num_drones - 1} peers"
        )

    def stop(self) -> None:
        """Stop tracking by destroying subscriptions."""
        for sub in self._subscriptions:
            self.node.destroy_subscription(sub)
        self._subscriptions.clear()
        self._started = False
        logger.info("NeighborTracker stopped")

    def _on_drone_state(self, drone_id: int, msg) -> None:
        """Handle incoming DroneState message from a peer."""
        now = time.time()

        # Check if this is a new neighbor
        is_new = drone_id not in self._known_neighbors

        # Update state
        neighbor = self._neighbors.get(drone_id)
        if neighbor is None:
            neighbor = NeighborState(drone_id=drone_id)
            self._neighbors[drone_id] = neighbor

        neighbor.position_ned = tuple(msg.position_ned)
        neighbor.velocity_ned = tuple(msg.velocity_ned)
        neighbor.yaw_deg = msg.yaw_deg
        neighbor.state = msg.state
        neighbor.battery_percent = msg.battery_percent
        neighbor.is_healthy = msg.is_healthy
        neighbor.last_update_time = now
        neighbor.is_stale = False

        # Fire callback for new neighbor
        if is_new:
            self._known_neighbors.add(drone_id)
            for callback in self._on_neighbor_joined:
                try:
                    callback(drone_id)
                except Exception as e:
                    logger.error(f"Error in on_neighbor_joined callback: {e}")

    def check_timeouts(self) -> list[int]:
        """
        Check for stale/failed neighbors.

        Returns:
            List of drone IDs that have timed out since last check.
        """
        now = time.time()
        newly_lost = []

        for drone_id, neighbor in self._neighbors.items():
            if neighbor.last_update_time == 0:
                continue  # Never received data

            age = now - neighbor.last_update_time

            # Check for stale (warning level)
            if age > self.config.stale_timeout and not neighbor.is_stale:
                neighbor.is_stale = True
                logger.warning(f"Neighbor {drone_id} is stale (no update for {age:.1f}s)")

            # Check for failed (lost level)
            if age > self.config.failed_timeout and drone_id in self._known_neighbors:
                self._known_neighbors.discard(drone_id)
                newly_lost.append(drone_id)
                logger.error(f"Neighbor {drone_id} lost (no update for {age:.1f}s)")

                for callback in self._on_neighbor_lost:
                    try:
                        callback(drone_id)
                    except Exception as e:
                        logger.error(f"Error in on_neighbor_lost callback: {e}")

        return newly_lost

    def get_neighbor(self, drone_id: int) -> NeighborState | None:
        """Get state of a specific neighbor."""
        return self._neighbors.get(drone_id)

    def get_all_neighbors(self) -> dict[int, NeighborState]:
        """Get states of all neighbors."""
        return self._neighbors.copy()

    def get_active_neighbors(self) -> dict[int, NeighborState]:
        """Get only non-stale neighbors."""
        return {
            did: state
            for did, state in self._neighbors.items()
            if not state.is_stale and did in self._known_neighbors
        }

    def predict_collision(
        self,
        own_position: tuple[float, float, float],
        own_velocity: tuple[float, float, float],
    ) -> CollisionWarning | None:
        """
        Predict collision with any neighbor.

        Uses linear extrapolation of positions over the collision horizon.

        Args:
            own_position: Own position in NED frame
            own_velocity: Own velocity in NED frame

        Returns:
            CollisionWarning for closest predicted collision, or None if safe.
        """
        closest_warning: CollisionWarning | None = None
        closest_time = float("inf")

        for drone_id, neighbor in self.get_active_neighbors().items():
            warning = self._check_collision_with_neighbor(
                own_position, own_velocity, neighbor
            )
            if warning and warning.time_to_collision < closest_time:
                closest_warning = warning
                closest_time = warning.time_to_collision

        # Fire callback if collision predicted
        if closest_warning:
            for callback in self._on_collision_warning:
                try:
                    callback(closest_warning)
                except Exception as e:
                    logger.error(f"Error in on_collision_warning callback: {e}")

        return closest_warning

    def _check_collision_with_neighbor(
        self,
        own_pos: tuple[float, float, float],
        own_vel: tuple[float, float, float],
        neighbor: NeighborState,
    ) -> CollisionWarning | None:
        """Check for collision with a single neighbor."""
        # Relative position and velocity
        rel_pos = (
            neighbor.position_ned[0] - own_pos[0],
            neighbor.position_ned[1] - own_pos[1],
            neighbor.position_ned[2] - own_pos[2],
        )
        rel_vel = (
            neighbor.velocity_ned[0] - own_vel[0],
            neighbor.velocity_ned[1] - own_vel[1],
            neighbor.velocity_ned[2] - own_vel[2],
        )

        # Current distance
        current_dist = math.sqrt(sum(p**2 for p in rel_pos))

        # If already too close, immediate warning
        if current_dist < self.config.safe_distance:
            avoidance = self._calculate_avoidance_vector(rel_pos)
            return CollisionWarning(
                other_drone_id=neighbor.drone_id,
                time_to_collision=0.0,
                min_distance=current_dist,
                avoidance_vector=avoidance,
            )

        # Time to closest approach (using dot product method)
        rel_vel_sq = sum(v**2 for v in rel_vel)
        if rel_vel_sq < 0.01:  # Effectively not moving relative to each other
            return None

        # t_min = -dot(rel_pos, rel_vel) / |rel_vel|^2
        dot_pv = sum(p * v for p, v in zip(rel_pos, rel_vel))
        t_min = -dot_pv / rel_vel_sq

        # Only consider future collisions within horizon
        if t_min < 0 or t_min > self.config.collision_horizon:
            return None

        # Distance at closest approach
        min_pos = tuple(p + v * t_min for p, v in zip(rel_pos, rel_vel))
        min_dist = math.sqrt(sum(p**2 for p in min_pos))

        if min_dist < self.config.safe_distance:
            avoidance = self._calculate_avoidance_vector(rel_pos)
            return CollisionWarning(
                other_drone_id=neighbor.drone_id,
                time_to_collision=t_min,
                min_distance=min_dist,
                avoidance_vector=avoidance,
            )

        return None

    def _calculate_avoidance_vector(
        self, rel_pos: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """
        Calculate avoidance velocity vector (move away from neighbor).

        Args:
            rel_pos: Relative position of neighbor (neighbor - self)

        Returns:
            Suggested velocity change in NED frame (normalized, points away from neighbor)
        """
        dist = math.sqrt(sum(p**2 for p in rel_pos))
        if dist < 0.01:
            # Neighbor at same position - move up
            return (0.0, 0.0, -1.0)

        # Move in opposite direction of neighbor
        return tuple(-p / dist for p in rel_pos)

    # Callback registration methods
    def on_neighbor_joined(self, callback: Callable[[int], None]) -> None:
        """Register callback for when a new neighbor is discovered."""
        self._on_neighbor_joined.append(callback)

    def on_neighbor_lost(self, callback: Callable[[int], None]) -> None:
        """Register callback for when a neighbor is lost."""
        self._on_neighbor_lost.append(callback)

    def on_collision_warning(self, callback: Callable[[CollisionWarning], None]) -> None:
        """Register callback for collision warnings."""
        self._on_collision_warning.append(callback)
