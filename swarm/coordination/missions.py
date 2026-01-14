"""Waypoint mission planning and execution for drone swarms.

Provides mission planning tools and execution tracking for coordinated
multi-drone waypoint missions.
"""

import time
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Tuple

logger = logging.getLogger(__name__)

# Position tuple: (north, east, down) in meters
PositionNED = Tuple[float, float, float]


class WaypointAction(Enum):
    """Actions to perform at a waypoint."""
    NONE = "none"
    HOVER = "hover"
    LOITER = "loiter"
    LAND = "land"
    RTL = "rtl"
    PHOTO = "photo"


class MissionExecutionMode(Enum):
    """How drones execute waypoints in a mission."""
    SEQUENTIAL = "sequential"      # All drones follow same path in sequence
    PARALLEL = "parallel"          # Each drone has independent path
    SYNCHRONIZED = "synchronized"  # All drones reach waypoint before proceeding


@dataclass
class Waypoint:
    """Single waypoint in a mission.

    Attributes:
        north: NED North coordinate (meters)
        east: NED East coordinate (meters)
        altitude: Altitude above ground (meters, positive = up)
        yaw: Heading at waypoint (degrees, 0 = North)
        speed: Approach speed (m/s)
        hold_time: Time to hold at waypoint (seconds)
        action: Action to perform at waypoint
        tolerance: Arrival tolerance radius (meters)
    """
    north: float
    east: float
    altitude: float
    yaw: float = 0.0
    speed: float = 5.0
    hold_time: float = 0.0
    action: WaypointAction = WaypointAction.NONE
    tolerance: float = 1.0

    @property
    def down(self) -> float:
        """NED Down coordinate (negative of altitude)."""
        return -self.altitude

    @property
    def position_ned(self) -> PositionNED:
        """Get position as NED tuple."""
        return (self.north, self.east, self.down)


@dataclass
class DroneMission:
    """Mission for a single drone.

    Attributes:
        drone_id: Drone identifier
        waypoints: List of waypoints for this drone
        current_waypoint_index: Index of current waypoint
        is_complete: Whether mission is finished
    """
    drone_id: int
    waypoints: List[Waypoint] = field(default_factory=list)
    current_waypoint_index: int = 0
    is_complete: bool = False

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Get current waypoint or None if complete."""
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

    @property
    def total_waypoints(self) -> int:
        """Total number of waypoints in mission."""
        return len(self.waypoints)

    @property
    def waypoints_remaining(self) -> int:
        """Number of waypoints remaining."""
        return max(0, len(self.waypoints) - self.current_waypoint_index)

    def advance(self) -> bool:
        """Move to next waypoint.

        Returns:
            True if more waypoints remain, False if mission complete
        """
        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self.waypoints):
            self.is_complete = True
            return False
        return True

    def reset(self) -> None:
        """Reset mission to beginning."""
        self.current_waypoint_index = 0
        self.is_complete = False


@dataclass
class SwarmMission:
    """Mission for the entire swarm.

    Attributes:
        name: Mission name/identifier
        description: Human-readable description
        execution_mode: How drones execute waypoints
        drone_missions: Per-drone mission data
        formation_at_waypoints: Whether to maintain formation between waypoints
    """
    name: str
    description: str = ""
    execution_mode: MissionExecutionMode = MissionExecutionMode.SYNCHRONIZED
    drone_missions: dict[int, DroneMission] = field(default_factory=dict)
    formation_at_waypoints: bool = True

    def is_complete(self) -> bool:
        """Check if all drone missions are complete."""
        if not self.drone_missions:
            return True
        return all(dm.is_complete for dm in self.drone_missions.values())

    def get_progress(self) -> float:
        """Get overall mission progress (0.0 to 1.0)."""
        if not self.drone_missions:
            return 1.0

        total_waypoints = sum(dm.total_waypoints for dm in self.drone_missions.values())
        if total_waypoints == 0:
            return 1.0

        completed = sum(dm.current_waypoint_index for dm in self.drone_missions.values())
        return completed / total_waypoints

    def reset(self) -> None:
        """Reset all drone missions to beginning."""
        for dm in self.drone_missions.values():
            dm.reset()


class MissionPlanner:
    """Creates swarm missions for common patterns."""

    @staticmethod
    def create_patrol_mission(
        waypoints: List[Tuple[float, float]],
        num_drones: int,
        altitude: float = 10.0,
        formation_positions: Optional[List[PositionNED]] = None,
        hold_time: float = 2.0,
    ) -> SwarmMission:
        """Create patrol mission where swarm visits waypoints as a group.

        The swarm maintains formation while moving between patrol waypoints.

        Args:
            waypoints: List of (north, east) patrol points
            num_drones: Number of drones in swarm
            altitude: Flight altitude (meters)
            formation_positions: Relative positions within formation.
                                If None, uses line formation.
            hold_time: Time to hold at each waypoint (seconds)

        Returns:
            SwarmMission for the patrol
        """
        mission = SwarmMission(
            name="patrol",
            description=f"Patrol {len(waypoints)} waypoints",
            execution_mode=MissionExecutionMode.SYNCHRONIZED,
            formation_at_waypoints=True,
        )

        # Default to line formation if not specified
        if formation_positions is None:
            formation_positions = [
                (0.0, i * 5.0, 0.0) for i in range(num_drones)
            ]

        for drone_id in range(num_drones):
            drone_wps = []

            # Get this drone's formation offset
            if drone_id < len(formation_positions):
                offset = formation_positions[drone_id]
            else:
                offset = (0.0, drone_id * 5.0, 0.0)

            for wp_n, wp_e in waypoints:
                drone_wps.append(Waypoint(
                    north=wp_n + offset[0],
                    east=wp_e + offset[1],
                    altitude=altitude + abs(offset[2]) if len(offset) > 2 else altitude,
                    hold_time=hold_time,
                ))

            mission.drone_missions[drone_id] = DroneMission(
                drone_id=drone_id,
                waypoints=drone_wps,
            )

        return mission

    @staticmethod
    def create_area_coverage_mission(
        bounds: Tuple[float, float, float, float],
        num_drones: int,
        altitude: float = 10.0,
        lane_spacing: float = 10.0,
    ) -> SwarmMission:
        """Create lawnmower pattern for area coverage.

        Divides area into vertical lanes, assigning one or more lanes
        per drone for efficient coverage.

        Args:
            bounds: Area bounds as (north_min, north_max, east_min, east_max)
            num_drones: Number of drones
            altitude: Flight altitude (meters)
            lane_spacing: Distance between lanes (meters)

        Returns:
            SwarmMission for area coverage
        """
        n_min, n_max, e_min, e_max = bounds

        mission = SwarmMission(
            name="area_coverage",
            description=f"Cover area ({n_min}, {e_min}) to ({n_max}, {e_max})",
            execution_mode=MissionExecutionMode.PARALLEL,
            formation_at_waypoints=False,
        )

        # Calculate lanes
        total_width = e_max - e_min
        num_lanes = max(1, int(total_width / lane_spacing))

        # Assign lanes to drones
        lanes_per_drone = max(1, num_lanes // num_drones)

        for drone_id in range(num_drones):
            drone_wps = []

            # Calculate this drone's lane range
            start_lane = drone_id * lanes_per_drone
            end_lane = min((drone_id + 1) * lanes_per_drone, num_lanes)

            if start_lane >= num_lanes:
                # No lanes for this drone
                mission.drone_missions[drone_id] = DroneMission(
                    drone_id=drone_id,
                    waypoints=[],
                )
                mission.drone_missions[drone_id].is_complete = True
                continue

            # Lawnmower pattern within assigned lanes
            going_north = True

            for lane in range(start_lane, end_lane):
                lane_e = e_min + lane * lane_spacing + lane_spacing / 2

                if going_north:
                    drone_wps.append(Waypoint(north=n_min, east=lane_e, altitude=altitude))
                    drone_wps.append(Waypoint(north=n_max, east=lane_e, altitude=altitude))
                else:
                    drone_wps.append(Waypoint(north=n_max, east=lane_e, altitude=altitude))
                    drone_wps.append(Waypoint(north=n_min, east=lane_e, altitude=altitude))

                going_north = not going_north

            mission.drone_missions[drone_id] = DroneMission(
                drone_id=drone_id,
                waypoints=drone_wps,
            )

        return mission

    @staticmethod
    def create_converge_mission(
        target: Tuple[float, float],
        num_drones: int,
        altitude: float = 10.0,
        start_positions: Optional[List[PositionNED]] = None,
    ) -> SwarmMission:
        """Create mission where all drones converge on a single point.

        Args:
            target: Target (north, east) coordinate
            num_drones: Number of drones
            altitude: Flight altitude (meters)
            start_positions: Optional current positions for path planning

        Returns:
            SwarmMission for convergence
        """
        mission = SwarmMission(
            name="converge",
            description=f"Converge on ({target[0]:.1f}, {target[1]:.1f})",
            execution_mode=MissionExecutionMode.SYNCHRONIZED,
            formation_at_waypoints=False,
        )

        for drone_id in range(num_drones):
            # Slight altitude offset to prevent collision at convergence point
            alt = altitude + drone_id * 2.0

            mission.drone_missions[drone_id] = DroneMission(
                drone_id=drone_id,
                waypoints=[
                    Waypoint(north=target[0], east=target[1], altitude=alt, hold_time=5.0)
                ],
            )

        return mission

    @staticmethod
    def create_orbit_mission(
        center: Tuple[float, float],
        radius: float,
        num_drones: int,
        altitude: float = 10.0,
        orbits: int = 2,
        points_per_orbit: int = 8,
    ) -> SwarmMission:
        """Create mission where drones orbit around a center point.

        Drones are spaced evenly around the circle and orbit together.

        Args:
            center: Center point (north, east)
            radius: Orbit radius (meters)
            num_drones: Number of drones
            altitude: Flight altitude (meters)
            orbits: Number of complete orbits
            points_per_orbit: Waypoints per orbit (more = smoother)

        Returns:
            SwarmMission for orbit
        """
        import math

        mission = SwarmMission(
            name="orbit",
            description=f"Orbit around ({center[0]:.1f}, {center[1]:.1f}), r={radius}m",
            execution_mode=MissionExecutionMode.SYNCHRONIZED,
            formation_at_waypoints=False,
        )

        total_points = orbits * points_per_orbit

        for drone_id in range(num_drones):
            drone_wps = []

            # Phase offset so drones are evenly distributed around circle
            phase_offset = 2 * math.pi * drone_id / num_drones
            alt = altitude + drone_id * 2.0

            for i in range(total_points):
                angle = 2 * math.pi * i / points_per_orbit + phase_offset

                north = center[0] + radius * math.cos(angle)
                east = center[1] + radius * math.sin(angle)

                drone_wps.append(Waypoint(north=north, east=east, altitude=alt))

            mission.drone_missions[drone_id] = DroneMission(
                drone_id=drone_id,
                waypoints=drone_wps,
            )

        return mission


class MissionExecutor:
    """Tracks and executes swarm missions.

    Handles waypoint arrival detection and synchronization between drones.
    """

    def __init__(
        self,
        mission: SwarmMission,
        position_tolerance: float = 1.0,
    ):
        """Initialize mission executor.

        Args:
            mission: Mission to execute
            position_tolerance: Override tolerance for waypoint arrival (meters)
        """
        self.mission = mission
        self.position_tolerance = position_tolerance
        self.is_running = False
        self.start_time: Optional[float] = None

        # Per-drone arrival tracking for synchronized mode
        self._arrived_at_waypoint: dict[int, bool] = {}
        self._waypoint_arrival_time: dict[int, float] = {}

    def start(self) -> None:
        """Start mission execution."""
        self.is_running = True
        self.start_time = time.time()
        self.mission.reset()

        # Initialize arrival tracking
        for drone_id in self.mission.drone_missions:
            self._arrived_at_waypoint[drone_id] = False
            self._waypoint_arrival_time[drone_id] = 0.0

        logger.info(f"Mission '{self.mission.name}' started")

    def stop(self) -> None:
        """Stop mission execution."""
        self.is_running = False
        logger.info(f"Mission '{self.mission.name}' stopped")

    def get_current_targets(self) -> dict[int, PositionNED]:
        """Get current target positions for all drones.

        Returns:
            Dict mapping drone_id to target (north, east, down)
        """
        targets = {}

        for drone_id, drone_mission in self.mission.drone_missions.items():
            if drone_mission.is_complete:
                continue

            wp = drone_mission.current_waypoint
            if wp:
                targets[drone_id] = wp.position_ned

        return targets

    def update_drone_position(
        self,
        drone_id: int,
        position: PositionNED
    ) -> Optional[Waypoint]:
        """Update drone position and check for waypoint arrival.

        Args:
            drone_id: Drone identifier
            position: Current (north, east, down) position

        Returns:
            Completed waypoint if drone just arrived, None otherwise
        """
        drone_mission = self.mission.drone_missions.get(drone_id)
        if not drone_mission or drone_mission.is_complete:
            return None

        wp = drone_mission.current_waypoint
        if not wp:
            return None

        # Check distance to waypoint
        tolerance = min(self.position_tolerance, wp.tolerance)
        dist = (
            (position[0] - wp.north) ** 2 +
            (position[1] - wp.east) ** 2 +
            (position[2] - wp.down) ** 2
        ) ** 0.5

        if dist <= tolerance:
            if not self._arrived_at_waypoint.get(drone_id, False):
                self._arrived_at_waypoint[drone_id] = True
                self._waypoint_arrival_time[drone_id] = time.time()
                logger.debug(f"Drone {drone_id} arrived at waypoint {drone_mission.current_waypoint_index}")

            # Check if we should advance
            if self._should_advance_waypoint(drone_id):
                completed_wp = wp
                drone_mission.advance()
                self._arrived_at_waypoint[drone_id] = False
                logger.info(
                    f"Drone {drone_id} completed waypoint, "
                    f"{drone_mission.waypoints_remaining} remaining"
                )
                return completed_wp

        return None

    def _should_advance_waypoint(self, drone_id: int) -> bool:
        """Check if drone should advance to next waypoint."""
        dm = self.mission.drone_missions.get(drone_id)
        if not dm:
            return False

        wp = dm.current_waypoint
        if not wp:
            return False

        # Check hold time
        arrival_time = self._waypoint_arrival_time.get(drone_id, 0)
        if time.time() - arrival_time < wp.hold_time:
            return False

        # For synchronized mode, wait for all active drones
        if self.mission.execution_mode == MissionExecutionMode.SYNCHRONIZED:
            all_arrived = all(
                self._arrived_at_waypoint.get(d_id, False)
                for d_id, dm in self.mission.drone_missions.items()
                if not dm.is_complete
            )

            if not all_arrived:
                return False

            # Also check all have satisfied hold time
            now = time.time()
            all_ready = True
            for d_id, dm in self.mission.drone_missions.items():
                if dm.is_complete:
                    continue
                d_wp = dm.current_waypoint
                if d_wp:
                    d_arrival = self._waypoint_arrival_time.get(d_id, 0)
                    if now - d_arrival < d_wp.hold_time:
                        all_ready = False
                        break

            if not all_ready:
                return False

            # Clear all arrival flags for synchronized mode
            for d_id in self.mission.drone_missions:
                self._arrived_at_waypoint[d_id] = False

        return True

    def get_elapsed_time(self) -> float:
        """Get elapsed time since mission start."""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
