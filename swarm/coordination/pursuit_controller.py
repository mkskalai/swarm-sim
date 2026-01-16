"""Multi-drone pursuit behavior for target tracking.

Provides pursuit coordination with multiple strategies:
- FOLLOW: All drones follow target
- INTERCEPT: Predict path and cut off
- SURROUND: Encircle target
- SHADOW: Follow at safe distance
"""

import math
import time
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict, Set, Tuple, Callable

from .missions import PositionNED, Waypoint
from .search_patterns import SearchPatternConfig, SearchPatternType

logger = logging.getLogger(__name__)


class PursuitStrategy(Enum):
    """Pursuit strategy types."""
    FOLLOW = "follow"        # All drones follow directly
    INTERCEPT = "intercept"  # Predict path and cut off
    SURROUND = "surround"    # Encircle target
    SHADOW = "shadow"        # Follow at distance


class PursuerRole(Enum):
    """Roles assigned to pursuing drones."""
    LEAD = "lead"            # Primary tracker
    FLANK_LEFT = "flank_left"
    FLANK_RIGHT = "flank_right"
    REAR = "rear"
    OVERWATCH = "overwatch"  # High altitude observation


@dataclass
class PursuitTarget:
    """Tracked target for pursuit.

    Attributes:
        target_id: Unique identifier (track ID from perception or assigned)
        position: Current position (north, east, down) in NED
        velocity: Velocity vector (vn, ve, vd) if known
        class_name: Object class from detection
        last_seen_time: Unix timestamp of last observation
        confidence: Detection confidence
        seen_by_drones: Set of drone IDs that have seen target
        heading: Estimated heading in degrees (0=North)
    """
    target_id: int
    position: PositionNED
    velocity: Optional[Tuple[float, float, float]] = None
    class_name: str = "unknown"
    last_seen_time: float = field(default_factory=time.time)
    confidence: float = 1.0
    seen_by_drones: Set[int] = field(default_factory=set)
    heading: float = 0.0

    @property
    def speed(self) -> float:
        """Get target speed from velocity."""
        if self.velocity is None:
            return 0.0
        vn, ve, vd = self.velocity
        return math.sqrt(vn**2 + ve**2 + vd**2)

    @property
    def horizontal_speed(self) -> float:
        """Get horizontal speed (ignoring vertical)."""
        if self.velocity is None:
            return 0.0
        vn, ve, _ = self.velocity
        return math.sqrt(vn**2 + ve**2)

    def predict_position(self, time_ahead: float) -> PositionNED:
        """Predict position at future time assuming constant velocity.

        Args:
            time_ahead: Seconds into future

        Returns:
            Predicted position (north, east, down)
        """
        if self.velocity is None:
            return self.position

        pn, pe, pd = self.position
        vn, ve, vd = self.velocity

        return (
            pn + vn * time_ahead,
            pe + ve * time_ahead,
            pd + vd * time_ahead,
        )


@dataclass
class PursuitConfig:
    """Configuration for pursuit behavior.

    Attributes:
        strategy: Pursuit strategy to use
        follow_distance: Distance behind target for FOLLOW/SHADOW
        surround_radius: Encirclement radius for SURROUND
        intercept_lead_time: Seconds ahead to predict for INTERCEPT
        min_altitude: Minimum pursuit altitude
        max_speed: Maximum pursuit speed (m/s)
        target_lost_timeout: Seconds before declaring target lost
        altitude_offset: Height above target altitude
        maintain_los: Try to maintain line of sight
        convergence_distance: Distance to switch from intercept to follow
    """
    strategy: PursuitStrategy = PursuitStrategy.SURROUND
    follow_distance: float = 10.0
    surround_radius: float = 15.0
    intercept_lead_time: float = 3.0
    min_altitude: float = 5.0
    max_speed: float = 10.0
    target_lost_timeout: float = 5.0
    altitude_offset: float = 5.0
    maintain_los: bool = True
    convergence_distance: float = 20.0


@dataclass
class PursuerState:
    """State of a pursuing drone.

    Attributes:
        drone_id: Drone identifier
        role: Assigned pursuit role
        target_position: Assigned target position
        current_position: Current drone position
        is_active: Whether actively pursuing
        time_on_target: Seconds target has been in view
    """
    drone_id: int
    role: PursuerRole = PursuerRole.LEAD
    target_position: Optional[PositionNED] = None
    current_position: Optional[PositionNED] = None
    is_active: bool = True
    time_on_target: float = 0.0


class PursuitController:
    """Multi-drone pursuit coordination.

    Coordinates multiple drones to track and follow a moving target
    using various strategies.

    Example:
        controller = PursuitController(
            num_drones=4,
            config=PursuitConfig(strategy=PursuitStrategy.SURROUND),
        )

        # Set target from detection
        controller.set_target(PursuitTarget(
            target_id=1,
            position=(50.0, 30.0, -1.0),
            velocity=(2.0, 1.0, 0.0),
            class_name="person",
        ))

        # In main loop
        controller.assign_pursuers([0, 1, 2, 3])
        targets = controller.get_pursuit_targets(drone_positions)
    """

    def __init__(
        self,
        num_drones: int,
        config: PursuitConfig,
        detection_tracker=None,
    ):
        """Initialize pursuit controller.

        Args:
            num_drones: Number of drones in swarm
            config: Pursuit configuration
            detection_tracker: Optional DetectionTracker for perception integration
        """
        self._num_drones = num_drones
        self._config = config
        self._detection_tracker = detection_tracker

        self._target: Optional[PursuitTarget] = None
        self._pursuer_states: Dict[int, PursuerState] = {}
        self._active_pursuers: Set[int] = set()

        # History for velocity estimation
        self._position_history: List[Tuple[float, PositionNED]] = []
        self._max_history = 10

        # Callbacks
        self._on_target_acquired: List[Callable[[PursuitTarget], None]] = []
        self._on_target_lost: List[Callable[[PursuitTarget], None]] = []
        self._on_target_reached: List[Callable[[int], None]] = []

        logger.info(f"Pursuit controller initialized: strategy={config.strategy.value}")

    def set_target(self, target: PursuitTarget) -> None:
        """Set pursuit target explicitly.

        Args:
            target: Target to pursue
        """
        was_tracking = self._target is not None
        self._target = target
        self._position_history = [(target.last_seen_time, target.position)]

        if not was_tracking:
            logger.info(f"Target acquired: {target.class_name} at {target.position}")
            for callback in self._on_target_acquired:
                callback(target)

    def set_target_from_detection(self, track_id: int) -> bool:
        """Set target from detection tracker.

        Args:
            track_id: Track ID from perception system

        Returns:
            True if target was found and set
        """
        if self._detection_tracker is None:
            logger.warning("No detection tracker configured")
            return False

        # Get detection by track ID
        detections = self._detection_tracker.get_all_detections()
        for det in detections:
            if det.track_id == track_id:
                self.set_target(PursuitTarget(
                    target_id=track_id,
                    position=det.estimated_position_ned,
                    class_name=det.class_name,
                    confidence=det.confidence,
                    last_seen_time=det.timestamp,
                    seen_by_drones={det.drone_id},
                ))
                return True

        logger.warning(f"Track {track_id} not found in detection tracker")
        return False

    def update_target_position(
        self,
        position: PositionNED,
        velocity: Optional[Tuple[float, float, float]] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        """Update target position (for simulated targets or manual updates).

        Args:
            position: New target position
            velocity: Optional velocity vector
            timestamp: Optional timestamp (defaults to now)
        """
        if self._target is None:
            return

        if timestamp is None:
            timestamp = time.time()

        # Update history
        self._position_history.append((timestamp, position))
        if len(self._position_history) > self._max_history:
            self._position_history.pop(0)

        # Estimate velocity if not provided
        if velocity is None and len(self._position_history) >= 2:
            velocity = self._estimate_velocity()

        self._target.position = position
        self._target.velocity = velocity
        self._target.last_seen_time = timestamp

        # Update heading from velocity
        if velocity and (velocity[0] != 0 or velocity[1] != 0):
            self._target.heading = math.degrees(math.atan2(velocity[1], velocity[0]))

    def _estimate_velocity(self) -> Optional[Tuple[float, float, float]]:
        """Estimate velocity from position history.

        Returns:
            Velocity vector or None
        """
        if len(self._position_history) < 2:
            return None

        # Use last two positions
        t1, p1 = self._position_history[-2]
        t2, p2 = self._position_history[-1]

        dt = t2 - t1
        if dt <= 0:
            return None

        return (
            (p2[0] - p1[0]) / dt,
            (p2[1] - p1[1]) / dt,
            (p2[2] - p1[2]) / dt,
        )

    def assign_pursuers(self, active_drones: List[int]) -> Dict[int, PursuerRole]:
        """Assign pursuit roles to drones.

        Args:
            active_drones: List of available drone IDs

        Returns:
            Dictionary mapping drone_id to assigned role
        """
        self._active_pursuers = set(active_drones)
        assignments: Dict[int, PursuerRole] = {}

        if not active_drones:
            return assignments

        num_pursuers = len(active_drones)

        if self._config.strategy == PursuitStrategy.FOLLOW:
            # All drones follow
            for i, drone_id in enumerate(active_drones):
                role = PursuerRole.LEAD if i == 0 else PursuerRole.REAR
                assignments[drone_id] = role

        elif self._config.strategy == PursuitStrategy.INTERCEPT:
            # First drone intercepts, others follow
            assignments[active_drones[0]] = PursuerRole.LEAD
            for drone_id in active_drones[1:]:
                assignments[drone_id] = PursuerRole.REAR

        elif self._config.strategy == PursuitStrategy.SURROUND:
            # Distribute roles around target
            roles = [PursuerRole.LEAD, PursuerRole.FLANK_LEFT,
                    PursuerRole.FLANK_RIGHT, PursuerRole.REAR]

            for i, drone_id in enumerate(active_drones):
                role = roles[i % len(roles)]
                assignments[drone_id] = role

        elif self._config.strategy == PursuitStrategy.SHADOW:
            # Lead with overwatch
            if num_pursuers >= 1:
                assignments[active_drones[0]] = PursuerRole.LEAD
            if num_pursuers >= 2:
                assignments[active_drones[1]] = PursuerRole.OVERWATCH
            for drone_id in active_drones[2:]:
                assignments[drone_id] = PursuerRole.REAR

        # Update pursuer states
        for drone_id, role in assignments.items():
            self._pursuer_states[drone_id] = PursuerState(
                drone_id=drone_id,
                role=role,
                is_active=True,
            )

        logger.info(f"Assigned {num_pursuers} pursuers: {assignments}")
        return assignments

    def get_pursuit_targets(
        self,
        drone_positions: Dict[int, PositionNED],
    ) -> Dict[int, PositionNED]:
        """Calculate target positions for each pursuing drone.

        Args:
            drone_positions: Current positions of all drones

        Returns:
            Dictionary mapping drone_id to target position
        """
        if self._target is None:
            return {}

        targets: Dict[int, PositionNED] = {}

        for drone_id in self._active_pursuers:
            if drone_id not in self._pursuer_states:
                continue

            state = self._pursuer_states[drone_id]
            role = state.role

            target_pos = self._calculate_pursuit_position(
                role=role,
                drone_id=drone_id,
                drone_position=drone_positions.get(drone_id),
            )

            if target_pos:
                targets[drone_id] = target_pos
                state.target_position = target_pos

        return targets

    def _calculate_pursuit_position(
        self,
        role: PursuerRole,
        drone_id: int,
        drone_position: Optional[PositionNED],
    ) -> Optional[PositionNED]:
        """Calculate pursuit position based on role and strategy.

        Args:
            role: Drone's assigned role
            drone_id: Drone identifier
            drone_position: Current drone position

        Returns:
            Target position or None
        """
        if self._target is None:
            return None

        target = self._target
        config = self._config
        tn, te, td = target.position

        # Base altitude (above target with offset)
        altitude = min(td, -config.min_altitude) - config.altitude_offset

        if config.strategy == PursuitStrategy.FOLLOW:
            # Follow behind target
            if target.velocity and target.horizontal_speed > 0.5:
                # Follow behind velocity vector
                vn, ve, _ = target.velocity
                speed = target.horizontal_speed
                offset_n = -config.follow_distance * vn / speed
                offset_e = -config.follow_distance * ve / speed
            else:
                # Default: follow from south
                offset_n = -config.follow_distance
                offset_e = 0.0

            return (tn + offset_n, te + offset_e, altitude)

        elif config.strategy == PursuitStrategy.INTERCEPT:
            if role == PursuerRole.LEAD:
                # Predict intercept point
                predicted = target.predict_position(config.intercept_lead_time)

                # If close enough, switch to follow
                if drone_position:
                    dist = self._distance_to(drone_position, predicted)
                    if dist < config.convergence_distance:
                        return (tn, te, altitude)

                return (predicted[0], predicted[1], altitude)
            else:
                # Others follow directly
                return (tn, te, altitude)

        elif config.strategy == PursuitStrategy.SURROUND:
            return self._calculate_surround_position(role, altitude)

        elif config.strategy == PursuitStrategy.SHADOW:
            if role == PursuerRole.OVERWATCH:
                # Higher altitude, directly above
                return (tn, te, altitude - 10.0)
            else:
                # Shadow from behind at distance
                if target.velocity and target.horizontal_speed > 0.5:
                    vn, ve, _ = target.velocity
                    speed = target.horizontal_speed
                    offset_n = -config.follow_distance * 1.5 * vn / speed
                    offset_e = -config.follow_distance * 1.5 * ve / speed
                else:
                    offset_n = -config.follow_distance * 1.5
                    offset_e = 0.0

                return (tn + offset_n, te + offset_e, altitude)

        return None

    def _calculate_surround_position(
        self,
        role: PursuerRole,
        altitude: float,
    ) -> Optional[PositionNED]:
        """Calculate position for surround strategy.

        Args:
            role: Drone's role
            altitude: Target altitude

        Returns:
            Position for surrounding target
        """
        if self._target is None:
            return None

        tn, te, _ = self._target.position
        radius = self._config.surround_radius

        # Calculate angle based on role
        # Use target heading to orient formation
        heading_rad = math.radians(self._target.heading)

        if role == PursuerRole.LEAD:
            # In front of target
            angle = heading_rad
        elif role == PursuerRole.FLANK_LEFT:
            angle = heading_rad + math.pi / 2
        elif role == PursuerRole.FLANK_RIGHT:
            angle = heading_rad - math.pi / 2
        elif role == PursuerRole.REAR:
            angle = heading_rad + math.pi
        else:
            angle = 0.0

        north = tn + radius * math.cos(angle)
        east = te + radius * math.sin(angle)

        return (north, east, altitude)

    def _distance_to(self, p1: PositionNED, p2: PositionNED) -> float:
        """Calculate horizontal distance between positions."""
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def check_target_visibility(
        self,
        drone_positions: Dict[int, PositionNED],
        drone_detections: Optional[Dict[int, List]] = None,
    ) -> bool:
        """Check if target is still visible to any pursuing drone.

        Args:
            drone_positions: Current drone positions
            drone_detections: Optional detection lists per drone

        Returns:
            True if target is visible
        """
        if self._target is None:
            return False

        current_time = time.time()

        # Check timeout
        if current_time - self._target.last_seen_time > self._config.target_lost_timeout:
            return False

        # If we have detection tracker, check for matching detections
        if self._detection_tracker:
            detections = self._detection_tracker.get_all_detections()
            for det in detections:
                if det.track_id == self._target.target_id:
                    # Update target from detection
                    self.update_target_position(
                        det.estimated_position_ned,
                        timestamp=det.timestamp,
                    )
                    self._target.seen_by_drones.add(det.drone_id)
                    return True

        # Fall back to timeout only
        return current_time - self._target.last_seen_time <= self._config.target_lost_timeout

    def handle_target_lost(self) -> Optional[SearchPatternConfig]:
        """Handle when pursuit target is lost.

        Returns search pattern configuration for re-acquisition search.

        Returns:
            SearchPatternConfig for searching or None
        """
        if self._target is None:
            return None

        lost_target = self._target
        logger.info(f"Target lost: {lost_target.class_name} "
                   f"last seen at {lost_target.position}")

        # Notify callbacks
        for callback in self._on_target_lost:
            callback(lost_target)

        # Create expanding square search from last known position
        last_pos = lost_target.position

        # Predict where target might have gone
        if lost_target.velocity and lost_target.horizontal_speed > 0.5:
            time_since = time.time() - lost_target.last_seen_time
            predicted = lost_target.predict_position(time_since)
            search_center = (predicted[0], predicted[1])
        else:
            search_center = (last_pos[0], last_pos[1])

        # Calculate search radius based on time lost and target speed
        time_lost = time.time() - lost_target.last_seen_time
        max_speed = lost_target.horizontal_speed if lost_target.velocity else 5.0
        search_radius = max(30.0, max_speed * time_lost * 1.5)

        self._target = None

        return SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=search_center,
            max_size=search_radius,
            step_size=10.0,
            altitude=abs(last_pos[2]) + self._config.altitude_offset,
        )

    def clear_target(self) -> None:
        """Clear current target."""
        self._target = None
        self._position_history = []

    @property
    def is_tracking(self) -> bool:
        """Whether actively tracking a target."""
        return self._target is not None

    @property
    def target(self) -> Optional[PursuitTarget]:
        """Current pursuit target."""
        return self._target

    @property
    def target_position(self) -> Optional[PositionNED]:
        """Current target position."""
        return self._target.position if self._target else None

    @property
    def active_pursuers(self) -> Set[int]:
        """Set of active pursuer drone IDs."""
        return self._active_pursuers.copy()

    def get_pursuer_state(self, drone_id: int) -> Optional[PursuerState]:
        """Get state of specific pursuer."""
        return self._pursuer_states.get(drone_id)

    def on_target_acquired(self, callback: Callable[[PursuitTarget], None]) -> None:
        """Register callback for target acquisition."""
        self._on_target_acquired.append(callback)

    def on_target_lost(self, callback: Callable[[PursuitTarget], None]) -> None:
        """Register callback for target loss."""
        self._on_target_lost.append(callback)


class SimulatedTarget:
    """Simulated moving target for pursuit testing.

    Creates a target that follows waypoints at specified speed.

    Example:
        target = SimulatedTarget(
            start_position=(0.0, 0.0, -1.5),
            waypoints=[(50.0, 0.0, -1.5), (50.0, 50.0, -1.5)],
            speed=3.0,
        )

        # In main loop
        target.update(dt)
        current_pos = target.position
    """

    def __init__(
        self,
        start_position: PositionNED,
        waypoints: List[PositionNED],
        speed: float = 5.0,
        loop: bool = False,
    ):
        """Initialize simulated target.

        Args:
            start_position: Starting position
            waypoints: List of waypoints to visit
            speed: Movement speed (m/s)
            loop: Whether to loop through waypoints
        """
        self._position = start_position
        self._waypoints = waypoints
        self._speed = speed
        self._loop = loop

        self._current_waypoint_idx = 0
        self._velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._is_complete = False

    def update(self, dt: float) -> None:
        """Update target position based on time delta.

        Args:
            dt: Time delta in seconds
        """
        if self._is_complete or not self._waypoints:
            self._velocity = (0.0, 0.0, 0.0)
            return

        target_wp = self._waypoints[self._current_waypoint_idx]

        # Calculate direction to waypoint
        dn = target_wp[0] - self._position[0]
        de = target_wp[1] - self._position[1]
        dd = target_wp[2] - self._position[2]

        dist = math.sqrt(dn**2 + de**2 + dd**2)

        if dist < 0.5:  # Reached waypoint
            self._current_waypoint_idx += 1

            if self._current_waypoint_idx >= len(self._waypoints):
                if self._loop:
                    self._current_waypoint_idx = 0
                else:
                    self._is_complete = True
                    self._velocity = (0.0, 0.0, 0.0)
                    return

            return

        # Normalize direction and apply speed
        factor = self._speed / dist
        vn = dn * factor
        ve = de * factor
        vd = dd * factor

        self._velocity = (vn, ve, vd)

        # Update position
        self._position = (
            self._position[0] + vn * dt,
            self._position[1] + ve * dt,
            self._position[2] + vd * dt,
        )

    @property
    def position(self) -> PositionNED:
        """Current target position."""
        return self._position

    @property
    def velocity(self) -> Tuple[float, float, float]:
        """Current target velocity."""
        return self._velocity

    @property
    def heading(self) -> float:
        """Current heading in degrees."""
        vn, ve, _ = self._velocity
        if vn == 0 and ve == 0:
            return 0.0
        return math.degrees(math.atan2(ve, vn))

    @property
    def is_complete(self) -> bool:
        """Whether target has reached final waypoint."""
        return self._is_complete

    def to_pursuit_target(self, target_id: int = 1) -> PursuitTarget:
        """Convert to PursuitTarget for controller.

        Args:
            target_id: ID to assign

        Returns:
            PursuitTarget instance
        """
        return PursuitTarget(
            target_id=target_id,
            position=self._position,
            velocity=self._velocity,
            class_name="simulated",
            last_seen_time=time.time(),
            confidence=1.0,
            heading=self.heading,
        )
