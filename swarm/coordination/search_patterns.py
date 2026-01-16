"""Search pattern algorithms for systematic area coverage.

Provides search pattern generation (spiral, lawnmower, expanding square)
and adaptive search control with dynamic reallocation when drones fail.
"""

import math
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict, Tuple, Callable, Set

from .missions import Waypoint, WaypointAction, PositionNED

logger = logging.getLogger(__name__)


class SearchPatternType(Enum):
    """Types of search patterns."""
    LAWNMOWER = "lawnmower"          # Boustrophedon (back-and-forth)
    SPIRAL = "spiral"                # Outward spiral from center
    EXPANDING_SQUARE = "expanding_square"  # Square spiral pattern
    SECTOR = "sector"                # Pie-slice sectors


@dataclass
class SearchPatternConfig:
    """Configuration for a search pattern.

    Attributes:
        pattern_type: Type of search pattern
        bounds: Area bounds (n_min, n_max, e_min, e_max) for lawnmower/sector
        center: Center point (north, east) for spiral/expanding square
        max_radius: Maximum radius for spiral pattern
        max_size: Maximum dimension for expanding square
        altitude: Search altitude (meters)
        lane_spacing: Spacing between parallel lanes (lawnmower)
        spiral_spacing: Distance between spiral arms
        step_size: Step increment for expanding square
        direction: Primary search direction ("north_south" or "east_west")
        clockwise: Spiral direction
        start_position: Optional starting position
    """
    pattern_type: SearchPatternType
    bounds: Optional[Tuple[float, float, float, float]] = None  # n_min, n_max, e_min, e_max
    center: Optional[Tuple[float, float]] = None
    max_radius: float = 100.0
    max_size: float = 100.0
    altitude: float = 15.0
    lane_spacing: float = 10.0
    spiral_spacing: float = 8.0
    step_size: float = 10.0
    direction: str = "north_south"
    clockwise: bool = True
    start_position: Optional[PositionNED] = None

    def __post_init__(self):
        """Validate configuration."""
        if self.pattern_type == SearchPatternType.LAWNMOWER:
            if self.bounds is None:
                raise ValueError("Lawnmower pattern requires bounds")
        elif self.pattern_type in (SearchPatternType.SPIRAL, SearchPatternType.EXPANDING_SQUARE):
            if self.center is None:
                raise ValueError(f"{self.pattern_type.value} pattern requires center")


@dataclass
class SearchProgress:
    """Tracks search progress for a drone.

    Attributes:
        drone_id: Drone identifier
        assigned_waypoints: Waypoints assigned to this drone
        current_index: Current waypoint index
        completed_waypoints: Number of waypoints completed
        coverage_area: Estimated area covered (square meters)
        detections_in_area: Number of detections made
    """
    drone_id: int
    assigned_waypoints: List[Waypoint] = field(default_factory=list)
    current_index: int = 0
    completed_waypoints: int = 0
    coverage_area: float = 0.0
    detections_in_area: int = 0

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Get current target waypoint."""
        if self.current_index < len(self.assigned_waypoints):
            return self.assigned_waypoints[self.current_index]
        return None

    @property
    def remaining_waypoints(self) -> List[Waypoint]:
        """Get remaining waypoints."""
        return self.assigned_waypoints[self.current_index:]

    @property
    def is_complete(self) -> bool:
        """Check if search is complete."""
        return self.current_index >= len(self.assigned_waypoints)

    def advance(self) -> bool:
        """Advance to next waypoint.

        Returns:
            True if more waypoints remain
        """
        if self.current_index < len(self.assigned_waypoints):
            self.completed_waypoints += 1
            self.current_index += 1
        return not self.is_complete


class SearchPatternGenerator:
    """Generates waypoint sequences for various search patterns."""

    @staticmethod
    def generate(config: SearchPatternConfig) -> List[Waypoint]:
        """Generate waypoints for the specified pattern.

        Args:
            config: Search pattern configuration

        Returns:
            List of waypoints for the search pattern
        """
        if config.pattern_type == SearchPatternType.LAWNMOWER:
            return SearchPatternGenerator.generate_lawnmower(
                bounds=config.bounds,
                lane_spacing=config.lane_spacing,
                altitude=config.altitude,
                direction=config.direction,
            )
        elif config.pattern_type == SearchPatternType.SPIRAL:
            return SearchPatternGenerator.generate_spiral(
                center=config.center,
                max_radius=config.max_radius,
                spacing=config.spiral_spacing,
                altitude=config.altitude,
                clockwise=config.clockwise,
            )
        elif config.pattern_type == SearchPatternType.EXPANDING_SQUARE:
            return SearchPatternGenerator.generate_expanding_square(
                center=config.center,
                max_size=config.max_size,
                step_size=config.step_size,
                altitude=config.altitude,
            )
        elif config.pattern_type == SearchPatternType.SECTOR:
            return SearchPatternGenerator.generate_sector(
                center=config.center or (0.0, 0.0),
                max_radius=config.max_radius,
                start_angle=0.0,
                end_angle=360.0,
                altitude=config.altitude,
                spacing=config.lane_spacing,
            )
        else:
            raise ValueError(f"Unknown pattern type: {config.pattern_type}")

    @staticmethod
    def generate_lawnmower(
        bounds: Tuple[float, float, float, float],
        lane_spacing: float = 10.0,
        altitude: float = 15.0,
        direction: str = "north_south",
    ) -> List[Waypoint]:
        """Generate lawnmower (boustrophedon) search pattern.

        Creates back-and-forth pattern covering rectangular area.

        Args:
            bounds: (n_min, n_max, e_min, e_max) in meters
            lane_spacing: Distance between parallel lanes
            altitude: Search altitude
            direction: "north_south" or "east_west"

        Returns:
            List of waypoints
        """
        n_min, n_max, e_min, e_max = bounds
        waypoints = []

        if direction == "north_south":
            # Lanes run north-south, move east between lanes
            num_lanes = max(1, int((e_max - e_min) / lane_spacing))
            going_north = True

            for i in range(num_lanes + 1):
                lane_e = e_min + i * lane_spacing
                if lane_e > e_max:
                    lane_e = e_max

                if going_north:
                    waypoints.append(Waypoint(north=n_min, east=lane_e, altitude=altitude))
                    waypoints.append(Waypoint(north=n_max, east=lane_e, altitude=altitude))
                else:
                    waypoints.append(Waypoint(north=n_max, east=lane_e, altitude=altitude))
                    waypoints.append(Waypoint(north=n_min, east=lane_e, altitude=altitude))

                going_north = not going_north
        else:
            # Lanes run east-west, move north between lanes
            num_lanes = max(1, int((n_max - n_min) / lane_spacing))
            going_east = True

            for i in range(num_lanes + 1):
                lane_n = n_min + i * lane_spacing
                if lane_n > n_max:
                    lane_n = n_max

                if going_east:
                    waypoints.append(Waypoint(north=lane_n, east=e_min, altitude=altitude))
                    waypoints.append(Waypoint(north=lane_n, east=e_max, altitude=altitude))
                else:
                    waypoints.append(Waypoint(north=lane_n, east=e_max, altitude=altitude))
                    waypoints.append(Waypoint(north=lane_n, east=e_min, altitude=altitude))

                going_east = not going_east

        return waypoints

    @staticmethod
    def generate_spiral(
        center: Tuple[float, float],
        max_radius: float,
        spacing: float = 8.0,
        altitude: float = 15.0,
        clockwise: bool = True,
        points_per_revolution: int = 16,
    ) -> List[Waypoint]:
        """Generate outward spiral search pattern.

        Starts at center and spirals outward with specified spacing.

        Args:
            center: (north, east) center point
            max_radius: Maximum spiral radius
            spacing: Distance between spiral arms
            altitude: Search altitude
            clockwise: Spiral direction
            points_per_revolution: Waypoints per full circle

        Returns:
            List of waypoints
        """
        waypoints = []
        center_n, center_e = center

        # Start at center
        waypoints.append(Waypoint(north=center_n, east=center_e, altitude=altitude))

        # Calculate number of revolutions needed
        num_revolutions = max_radius / spacing
        total_points = int(num_revolutions * points_per_revolution)

        direction = -1 if clockwise else 1

        for i in range(1, total_points + 1):
            # Archimedes spiral: r = a + b*theta
            theta = direction * 2 * math.pi * i / points_per_revolution
            radius = spacing * i / points_per_revolution

            if radius > max_radius:
                break

            north = center_n + radius * math.cos(theta)
            east = center_e + radius * math.sin(theta)

            waypoints.append(Waypoint(north=north, east=east, altitude=altitude))

        return waypoints

    @staticmethod
    def generate_expanding_square(
        center: Tuple[float, float],
        max_size: float,
        step_size: float = 10.0,
        altitude: float = 15.0,
    ) -> List[Waypoint]:
        """Generate expanding square search pattern.

        Creates square spiral pattern expanding from center.
        Good for searching from last known position.

        Args:
            center: (north, east) center point
            max_size: Maximum square side length
            step_size: Distance to expand each leg
            altitude: Search altitude

        Returns:
            List of waypoints
        """
        waypoints = []
        center_n, center_e = center

        # Start at center
        current_n, current_e = center_n, center_e
        waypoints.append(Waypoint(north=current_n, east=current_e, altitude=altitude))

        # Expanding square: N, E, S, S, W, W, N, N, N, E, E, E, ...
        # Direction order: North, East, South, West
        directions = [
            (1, 0),   # North
            (0, 1),   # East
            (-1, 0),  # South
            (0, -1),  # West
        ]

        leg_length = step_size
        dir_index = 0
        moves_at_length = 0

        while leg_length <= max_size * 2:
            dn, de = directions[dir_index % 4]

            current_n += dn * step_size
            current_e += de * step_size

            # Check bounds
            if (abs(current_n - center_n) > max_size or
                abs(current_e - center_e) > max_size):
                break

            waypoints.append(Waypoint(north=current_n, east=current_e, altitude=altitude))

            moves_at_length += 1

            # After 2 moves, increase leg length
            if moves_at_length >= leg_length / step_size:
                dir_index += 1
                moves_at_length = 0

                # Every 2 direction changes, increase leg length
                if dir_index % 2 == 0:
                    leg_length += step_size

        return waypoints

    @staticmethod
    def generate_sector(
        center: Tuple[float, float],
        max_radius: float,
        start_angle: float = 0.0,
        end_angle: float = 90.0,
        altitude: float = 15.0,
        spacing: float = 10.0,
    ) -> List[Waypoint]:
        """Generate sector (pie-slice) search pattern.

        Sweeps radially within angular bounds.

        Args:
            center: (north, east) center point
            max_radius: Sector radius
            start_angle: Starting angle in degrees (0 = North)
            end_angle: Ending angle in degrees
            altitude: Search altitude
            spacing: Distance between radial sweeps

        Returns:
            List of waypoints
        """
        waypoints = []
        center_n, center_e = center

        # Convert to radians, adjust for NED (0 = North, clockwise)
        start_rad = math.radians(90 - start_angle)
        end_rad = math.radians(90 - end_angle)

        num_rings = max(1, int(max_radius / spacing))

        for ring in range(num_rings + 1):
            radius = spacing * ring
            if radius > max_radius:
                radius = max_radius

            # Angular resolution based on radius
            if radius > 0:
                arc_length = abs(end_rad - start_rad) * radius
                num_points = max(2, int(arc_length / spacing))
            else:
                num_points = 1

            # Alternate direction for efficiency
            if ring % 2 == 0:
                angles = [start_rad + (end_rad - start_rad) * i / max(1, num_points - 1)
                         for i in range(num_points)]
            else:
                angles = [end_rad + (start_rad - end_rad) * i / max(1, num_points - 1)
                         for i in range(num_points)]

            for angle in angles:
                if radius == 0:
                    north, east = center_n, center_e
                else:
                    north = center_n + radius * math.sin(angle)
                    east = center_e + radius * math.cos(angle)

                waypoints.append(Waypoint(north=north, east=east, altitude=altitude))

        return waypoints


class AdaptiveSearchController:
    """Manages distributed search with dynamic reallocation.

    Handles:
    - Initial waypoint distribution among drones
    - Reallocation when drones fail
    - Detection-triggered pattern adjustments
    - Coverage tracking

    Example:
        controller = AdaptiveSearchController(
            num_drones=4,
            pattern_config=SearchPatternConfig(
                pattern_type=SearchPatternType.EXPANDING_SQUARE,
                center=(0, 0),
                max_size=100,
            ),
        )

        # Initialize search
        assignments = controller.initialize_search([0, 1, 2, 3])

        # In main loop
        targets = controller.get_current_targets()
        for drone_id, position in drone_positions.items():
            if controller.update_progress(drone_id, position):
                print(f"Drone {drone_id} reached waypoint")

        # On drone failure
        if drone_failed:
            new_assignments = controller.on_drone_failure(failed_drone_id)
    """

    def __init__(
        self,
        num_drones: int,
        pattern_config: SearchPatternConfig,
        failure_handler=None,
        arrival_tolerance: float = 2.0,
    ):
        """Initialize adaptive search controller.

        Args:
            num_drones: Number of drones in swarm
            pattern_config: Search pattern configuration
            failure_handler: Optional FailureHandler for integration
            arrival_tolerance: Distance to consider waypoint reached (meters)
        """
        self._num_drones = num_drones
        self._pattern_config = pattern_config
        self._failure_handler = failure_handler
        self._arrival_tolerance = arrival_tolerance

        self._progress: Dict[int, SearchProgress] = {}
        self._active_drones: Set[int] = set()
        self._all_waypoints: List[Waypoint] = []
        self._is_initialized = False

        # Callbacks
        self._on_waypoint_reached: List[Callable[[int, Waypoint], None]] = []
        self._on_search_complete: List[Callable[[], None]] = []
        self._on_detection: List[Callable] = []

    def initialize_search(self, active_drones: List[int]) -> Dict[int, List[Waypoint]]:
        """Initialize search and distribute waypoints among drones.

        Args:
            active_drones: List of drone IDs available for search

        Returns:
            Dictionary mapping drone_id to assigned waypoints
        """
        self._active_drones = set(active_drones)

        # Generate all waypoints
        self._all_waypoints = SearchPatternGenerator.generate(self._pattern_config)

        if not self._all_waypoints:
            logger.warning("No waypoints generated for search pattern")
            return {}

        # Distribute waypoints among drones
        assignments = self._distribute_waypoints(
            self._all_waypoints.copy(),
            list(self._active_drones)
        )

        # Initialize progress tracking
        for drone_id, waypoints in assignments.items():
            self._progress[drone_id] = SearchProgress(
                drone_id=drone_id,
                assigned_waypoints=waypoints,
            )

        self._is_initialized = True
        logger.info(f"Initialized search with {len(self._all_waypoints)} waypoints "
                   f"distributed among {len(active_drones)} drones")

        return assignments

    def _distribute_waypoints(
        self,
        waypoints: List[Waypoint],
        drone_ids: List[int],
    ) -> Dict[int, List[Waypoint]]:
        """Distribute waypoints among drones.

        Uses simple round-robin distribution with contiguous segments
        for efficiency.

        Args:
            waypoints: Waypoints to distribute
            drone_ids: Available drone IDs

        Returns:
            Dictionary mapping drone_id to waypoint list
        """
        if not drone_ids:
            return {}

        assignments: Dict[int, List[Waypoint]] = {d: [] for d in drone_ids}

        # Divide into contiguous segments
        num_drones = len(drone_ids)
        waypoints_per_drone = len(waypoints) // num_drones
        remainder = len(waypoints) % num_drones

        idx = 0
        for i, drone_id in enumerate(drone_ids):
            # Give one extra waypoint to first 'remainder' drones
            count = waypoints_per_drone + (1 if i < remainder else 0)
            assignments[drone_id] = waypoints[idx:idx + count]
            idx += count

        return assignments

    def on_drone_failure(self, drone_id: int) -> Dict[int, List[Waypoint]]:
        """Handle drone failure and reallocate remaining waypoints.

        Args:
            drone_id: ID of failed drone

        Returns:
            Updated waypoint assignments for remaining drones
        """
        if drone_id not in self._active_drones:
            return {}

        logger.info(f"Handling failure of drone {drone_id}, reallocating search")

        # Get remaining waypoints from failed drone
        remaining_waypoints = []
        if drone_id in self._progress:
            remaining_waypoints = self._progress[drone_id].remaining_waypoints
            del self._progress[drone_id]

        self._active_drones.discard(drone_id)

        if not self._active_drones:
            logger.warning("No active drones remaining for search")
            return {}

        if not remaining_waypoints:
            return {}

        # Redistribute remaining waypoints
        redistributed = self._distribute_waypoints(
            remaining_waypoints,
            list(self._active_drones)
        )

        # Append to existing assignments
        for d_id, new_wps in redistributed.items():
            if d_id in self._progress:
                self._progress[d_id].assigned_waypoints.extend(new_wps)
            else:
                self._progress[d_id] = SearchProgress(
                    drone_id=d_id,
                    assigned_waypoints=new_wps,
                )

        logger.info(f"Redistributed {len(remaining_waypoints)} waypoints "
                   f"among {len(self._active_drones)} remaining drones")

        return {d_id: self._progress[d_id].assigned_waypoints
                for d_id in self._active_drones}

    def on_detection(
        self,
        detection,
        investigating_drone: Optional[int] = None,
    ) -> Optional[Dict[int, List[Waypoint]]]:
        """Handle detection during search.

        Can optionally modify search pattern based on detection.
        Default behavior: continue search, just record detection.

        Args:
            detection: Detection object (from perception system)
            investigating_drone: Drone to detach for investigation

        Returns:
            Updated waypoint assignments if pattern changed, else None
        """
        # Record detection
        if investigating_drone and investigating_drone in self._progress:
            self._progress[investigating_drone].detections_in_area += 1

        # Notify callbacks
        for callback in self._on_detection:
            callback(detection, investigating_drone)

        # Default: don't modify pattern
        return None

    def get_current_targets(self) -> Dict[int, PositionNED]:
        """Get current target position for each active drone.

        Returns:
            Dictionary mapping drone_id to target NED position
        """
        targets = {}

        for drone_id in self._active_drones:
            if drone_id in self._progress:
                wp = self._progress[drone_id].current_waypoint
                if wp:
                    targets[drone_id] = wp.position_ned

        return targets

    def update_progress(self, drone_id: int, position: PositionNED) -> bool:
        """Update drone progress based on current position.

        Args:
            drone_id: Drone identifier
            position: Current position (north, east, down)

        Returns:
            True if waypoint was reached
        """
        if drone_id not in self._progress:
            return False

        progress = self._progress[drone_id]
        current_wp = progress.current_waypoint

        if not current_wp:
            return False

        # Check if arrived
        target = current_wp.position_ned
        dist = math.sqrt(
            (position[0] - target[0])**2 +
            (position[1] - target[1])**2
        )

        if dist <= self._arrival_tolerance:
            # Update coverage estimate
            progress.coverage_area += self._pattern_config.lane_spacing ** 2

            # Advance to next waypoint
            has_more = progress.advance()

            # Notify callbacks
            for callback in self._on_waypoint_reached:
                callback(drone_id, current_wp)

            # Check if search is complete
            if self.is_search_complete:
                for callback in self._on_search_complete:
                    callback()

            return True

        return False

    def get_progress(self, drone_id: int) -> Optional[SearchProgress]:
        """Get progress for specific drone."""
        return self._progress.get(drone_id)

    def get_all_progress(self) -> Dict[int, SearchProgress]:
        """Get progress for all drones."""
        return self._progress.copy()

    @property
    def is_search_complete(self) -> bool:
        """Check if all drones have completed their search."""
        if not self._is_initialized:
            return False
        return all(p.is_complete for p in self._progress.values())

    @property
    def coverage_stats(self) -> Dict[str, float]:
        """Get coverage statistics.

        Returns:
            Dictionary with coverage metrics
        """
        total_waypoints = sum(len(p.assigned_waypoints) for p in self._progress.values())
        completed_waypoints = sum(p.completed_waypoints for p in self._progress.values())
        total_coverage = sum(p.coverage_area for p in self._progress.values())
        total_detections = sum(p.detections_in_area for p in self._progress.values())

        return {
            "total_waypoints": total_waypoints,
            "completed_waypoints": completed_waypoints,
            "completion_percent": (completed_waypoints / total_waypoints * 100) if total_waypoints else 0,
            "coverage_area_m2": total_coverage,
            "total_detections": total_detections,
            "active_drones": len(self._active_drones),
        }

    def register_waypoint_callback(self, callback: Callable[[int, Waypoint], None]) -> None:
        """Register callback for waypoint reached events."""
        self._on_waypoint_reached.append(callback)

    def register_completion_callback(self, callback: Callable[[], None]) -> None:
        """Register callback for search completion."""
        self._on_search_complete.append(callback)

    def register_detection_callback(self, callback: Callable) -> None:
        """Register callback for detection events."""
        self._on_detection.append(callback)
