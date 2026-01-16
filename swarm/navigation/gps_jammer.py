"""Enhanced GPS denial simulation with zone-based and probabilistic jamming.

Extends basic GPS denial simulation with:
- Geographic denial zones (sphere, cylinder, box shapes)
- Probabilistic/intermittent denial
- Scheduled denial events
- Integration with terrain-based GPS shadowing
"""

import math
import time
import random
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict, Set, Tuple, Callable

from .position_source import PositionSource, GPSDenialSimulator

logger = logging.getLogger(__name__)

# Position type: (north, east, down) in meters
PositionNED = Tuple[float, float, float]


class DenialZoneShape(Enum):
    """Shape of GPS denial zone."""
    SPHERE = "sphere"        # Spherical zone
    CYLINDER = "cylinder"    # Vertical cylinder (infinite or bounded height)
    BOX = "box"             # Axis-aligned box


@dataclass
class GPSDenialZone:
    """Geographic zone where GPS is denied.

    Attributes:
        zone_id: Unique identifier
        center: Center position (north, east, down) in NED
        radius: Horizontal radius for sphere/cylinder (meters)
        shape: Zone shape
        probability: Denial probability (0.0-1.0) for intermittent jamming
        height: Height for cylinder (None = infinite)
        dimensions: (north, east, down) dimensions for box shape
        active: Whether zone is currently active
        name: Human-readable name
    """
    zone_id: int
    center: PositionNED
    radius: float = 20.0
    shape: DenialZoneShape = DenialZoneShape.SPHERE
    probability: float = 1.0
    height: Optional[float] = None
    dimensions: Optional[Tuple[float, float, float]] = None
    active: bool = True
    name: str = ""

    def contains(self, position: PositionNED) -> bool:
        """Check if position is within this zone.

        Args:
            position: Position to check (north, east, down)

        Returns:
            True if position is within zone
        """
        if not self.active:
            return False

        pn, pe, pd = position
        cn, ce, cd = self.center

        if self.shape == DenialZoneShape.SPHERE:
            dist = math.sqrt(
                (pn - cn)**2 + (pe - ce)**2 + (pd - cd)**2
            )
            return dist <= self.radius

        elif self.shape == DenialZoneShape.CYLINDER:
            # Horizontal distance
            h_dist = math.sqrt((pn - cn)**2 + (pe - ce)**2)
            if h_dist > self.radius:
                return False

            # Vertical check
            if self.height is not None:
                half_height = self.height / 2
                if abs(pd - cd) > half_height:
                    return False

            return True

        elif self.shape == DenialZoneShape.BOX:
            if self.dimensions is None:
                return False

            dn, de, dd = self.dimensions
            half_n, half_e, half_d = dn / 2, de / 2, dd / 2

            return (abs(pn - cn) <= half_n and
                    abs(pe - ce) <= half_e and
                    abs(pd - cd) <= half_d)

        return False

    def get_denial_probability(self, position: PositionNED) -> float:
        """Get GPS denial probability at position.

        Args:
            position: Position to check

        Returns:
            Probability of denial (0.0 to 1.0)
        """
        if not self.contains(position):
            return 0.0
        return self.probability


@dataclass
class ScheduledDenial:
    """Scheduled GPS denial event.

    Attributes:
        drone_id: Target drone (-1 for all)
        start_time: Unix timestamp to start denial
        duration: Duration of denial (seconds)
        executed: Whether denial has been started
        restored: Whether GPS has been restored
    """
    drone_id: int
    start_time: float
    duration: float
    executed: bool = False
    restored: bool = False

    @property
    def end_time(self) -> float:
        """End time of denial."""
        return self.start_time + self.duration

    def is_active(self, current_time: float) -> bool:
        """Check if denial is active at given time."""
        return self.start_time <= current_time < self.end_time


@dataclass
class GPSJammerConfig:
    """Configuration for GPS jammer.

    Attributes:
        zones: List of GPS denial zones
        global_probability: Random denial probability for any position
        update_rate: Rate to check positions (Hz)
        enable_p2p_relative: Enable P2P relative positioning when jammed
        random_seed: Seed for reproducible random denial
    """
    zones: List[GPSDenialZone] = field(default_factory=list)
    global_probability: float = 0.0
    update_rate: float = 1.0
    enable_p2p_relative: bool = True
    random_seed: Optional[int] = None


class GPSJammer:
    """Enhanced GPS denial simulation with zones and probability.

    Extends GPSDenialSimulator with:
    - Zone-based denial (drones entering zones lose GPS)
    - Probabilistic denial (intermittent jamming)
    - Scheduled denial events
    - Statistics tracking

    Example:
        # Create jammer with denial zone
        config = GPSJammerConfig(
            zones=[GPSDenialZone(
                zone_id=1,
                center=(50.0, 50.0, -10.0),
                radius=30.0,
                shape=DenialZoneShape.CYLINDER,
            )]
        )
        jammer = GPSJammer(config)

        # Register drones
        for drone_id, source in position_sources.items():
            jammer.register_position_source(drone_id, source)

        # In main loop
        jammed = jammer.update(drone_positions)
        if jammed:
            print(f"GPS denied for drones: {jammed}")
    """

    def __init__(self, config: GPSJammerConfig):
        """Initialize GPS jammer.

        Args:
            config: Jammer configuration
        """
        self._config = config
        self._base_simulator = GPSDenialSimulator()
        self._sources: Dict[int, PositionSource] = {}

        # Tracking
        self._jammed_drones: Set[int] = set()
        self._zone_jammed: Dict[int, int] = {}  # drone_id -> zone_id that jammed it
        self._scheduled_denials: List[ScheduledDenial] = []

        # Statistics
        self._total_denial_time: Dict[int, float] = {}  # drone_id -> seconds
        self._denial_start_time: Dict[int, float] = {}  # drone_id -> start timestamp
        self._zone_entry_count: Dict[int, int] = {}  # zone_id -> entry count

        # Random state
        if config.random_seed is not None:
            self._rng = random.Random(config.random_seed)
        else:
            self._rng = random.Random()

        # Timing
        self._last_update_time = 0.0
        self._update_interval = 1.0 / config.update_rate

        # Callbacks
        self._on_denial: List[Callable[[int], None]] = []
        self._on_restore: List[Callable[[int], None]] = []

        logger.info(f"GPS Jammer initialized with {len(config.zones)} denial zones")

    def register_position_source(self, drone_id: int, source: PositionSource) -> None:
        """Register a position source for GPS denial simulation.

        Args:
            drone_id: Drone identifier
            source: Position source to control
        """
        self._sources[drone_id] = source
        self._base_simulator.register_source(drone_id, source)
        self._total_denial_time[drone_id] = 0.0
        logger.debug(f"Registered drone {drone_id} with GPS jammer")

    def add_zone(self, zone: GPSDenialZone) -> None:
        """Add a GPS denial zone.

        Args:
            zone: Zone to add
        """
        self._config.zones.append(zone)
        self._zone_entry_count[zone.zone_id] = 0
        logger.info(f"Added GPS denial zone {zone.zone_id}: {zone.name or zone.shape.value} "
                   f"at {zone.center} r={zone.radius}m")

    def remove_zone(self, zone_id: int) -> bool:
        """Remove a GPS denial zone.

        Args:
            zone_id: ID of zone to remove

        Returns:
            True if zone was found and removed
        """
        for i, zone in enumerate(self._config.zones):
            if zone.zone_id == zone_id:
                del self._config.zones[i]
                logger.info(f"Removed GPS denial zone {zone_id}")
                return True
        return False

    def set_zone_active(self, zone_id: int, active: bool) -> bool:
        """Enable or disable a denial zone.

        Args:
            zone_id: Zone identifier
            active: Whether zone should be active

        Returns:
            True if zone was found
        """
        for zone in self._config.zones:
            if zone.zone_id == zone_id:
                zone.active = active
                logger.info(f"Zone {zone_id} {'activated' if active else 'deactivated'}")
                return True
        return False

    def schedule_denial(
        self,
        drone_id: int,
        start_time: float,
        duration: float,
    ) -> None:
        """Schedule GPS denial for a drone.

        Args:
            drone_id: Target drone (-1 for all drones)
            start_time: Unix timestamp to start denial
            duration: Duration in seconds
        """
        self._scheduled_denials.append(ScheduledDenial(
            drone_id=drone_id,
            start_time=start_time,
            duration=duration,
        ))
        logger.info(f"Scheduled GPS denial for drone {drone_id}: "
                   f"start={start_time:.1f}, duration={duration:.1f}s")

    def update(
        self,
        drone_positions: Dict[int, PositionNED],
        current_time: Optional[float] = None,
    ) -> Set[int]:
        """Update GPS denial state based on drone positions.

        Args:
            drone_positions: Dictionary mapping drone_id to NED position
            current_time: Current time (defaults to time.time())

        Returns:
            Set of drone IDs that are newly jammed
        """
        if current_time is None:
            current_time = time.time()

        # Rate limit updates
        if current_time - self._last_update_time < self._update_interval:
            return set()
        self._last_update_time = current_time

        newly_jammed: Set[int] = set()

        # Process scheduled denials
        self._process_scheduled_denials(current_time)

        # Check each drone position against zones
        for drone_id, position in drone_positions.items():
            if drone_id not in self._sources:
                continue

            was_jammed = drone_id in self._jammed_drones
            should_jam = self._check_denial(drone_id, position)

            if should_jam and not was_jammed:
                # Newly jammed
                self._deny_gps(drone_id)
                newly_jammed.add(drone_id)

            elif not should_jam and was_jammed:
                # Check if should restore (not in any zone, not scheduled)
                if self._can_restore(drone_id, current_time):
                    self._restore_gps(drone_id)

        return newly_jammed

    def _check_denial(self, drone_id: int, position: PositionNED) -> bool:
        """Check if GPS should be denied for drone at position.

        Args:
            drone_id: Drone identifier
            position: Current position

        Returns:
            True if GPS should be denied
        """
        # Check global random probability
        if self._config.global_probability > 0:
            if self._rng.random() < self._config.global_probability:
                return True

        # Check zones
        for zone in self._config.zones:
            prob = zone.get_denial_probability(position)
            if prob > 0:
                if prob >= 1.0 or self._rng.random() < prob:
                    self._zone_jammed[drone_id] = zone.zone_id
                    return True

        return False

    def _can_restore(self, drone_id: int, current_time: float) -> bool:
        """Check if GPS can be restored for drone.

        Args:
            drone_id: Drone identifier
            current_time: Current time

        Returns:
            True if GPS can be restored
        """
        # Check scheduled denials
        for denial in self._scheduled_denials:
            if denial.is_active(current_time):
                if denial.drone_id == -1 or denial.drone_id == drone_id:
                    return False
        return True

    def _process_scheduled_denials(self, current_time: float) -> None:
        """Process scheduled denial events.

        Args:
            current_time: Current time
        """
        for denial in self._scheduled_denials:
            if not denial.executed and current_time >= denial.start_time:
                # Start denial
                if denial.drone_id == -1:
                    for drone_id in self._sources:
                        if drone_id not in self._jammed_drones:
                            self._deny_gps(drone_id)
                elif denial.drone_id not in self._jammed_drones:
                    self._deny_gps(denial.drone_id)
                denial.executed = True

            elif denial.executed and not denial.restored and current_time >= denial.end_time:
                # End denial
                if denial.drone_id == -1:
                    for drone_id in list(self._jammed_drones):
                        if self._can_restore(drone_id, current_time):
                            self._restore_gps(drone_id)
                elif self._can_restore(denial.drone_id, current_time):
                    self._restore_gps(denial.drone_id)
                denial.restored = True

    def _deny_gps(self, drone_id: int) -> None:
        """Deny GPS for drone.

        Args:
            drone_id: Drone to deny
        """
        self._base_simulator.deny_gps(drone_id)
        self._jammed_drones.add(drone_id)
        self._denial_start_time[drone_id] = time.time()

        # Update zone entry count
        if drone_id in self._zone_jammed:
            zone_id = self._zone_jammed[drone_id]
            self._zone_entry_count[zone_id] = self._zone_entry_count.get(zone_id, 0) + 1

        logger.info(f"GPS denied for drone {drone_id}")

        for callback in self._on_denial:
            callback(drone_id)

    def _restore_gps(self, drone_id: int) -> None:
        """Restore GPS for drone.

        Args:
            drone_id: Drone to restore
        """
        self._base_simulator.restore_gps(drone_id)
        self._jammed_drones.discard(drone_id)
        self._zone_jammed.pop(drone_id, None)

        # Update total denial time
        if drone_id in self._denial_start_time:
            denial_duration = time.time() - self._denial_start_time[drone_id]
            self._total_denial_time[drone_id] = (
                self._total_denial_time.get(drone_id, 0.0) + denial_duration
            )
            del self._denial_start_time[drone_id]

        logger.info(f"GPS restored for drone {drone_id}")

        for callback in self._on_restore:
            callback(drone_id)

    def deny_gps_manual(self, drone_id: int) -> None:
        """Manually deny GPS for drone (bypasses zone checks).

        Args:
            drone_id: Drone to deny
        """
        if drone_id in self._sources and drone_id not in self._jammed_drones:
            self._deny_gps(drone_id)

    def restore_gps_manual(self, drone_id: int) -> None:
        """Manually restore GPS for drone.

        Args:
            drone_id: Drone to restore
        """
        if drone_id in self._jammed_drones:
            self._restore_gps(drone_id)

    def deny_gps_percentage(self, percentage: float) -> Set[int]:
        """Deny GPS for a percentage of drones.

        Useful for testing degraded operation scenarios.

        Args:
            percentage: Percentage of drones to deny (0-100)

        Returns:
            Set of drone IDs that were denied
        """
        if not self._sources:
            return set()

        available = [d for d in self._sources if d not in self._jammed_drones]
        num_to_deny = int(len(self._sources) * percentage / 100)
        num_to_deny = min(num_to_deny, len(available))

        to_deny = self._rng.sample(available, num_to_deny)

        for drone_id in to_deny:
            self._deny_gps(drone_id)

        logger.info(f"Denied GPS for {len(to_deny)} drones ({percentage}%)")
        return set(to_deny)

    def is_in_denial_zone(self, position: PositionNED) -> bool:
        """Check if position is within any GPS denial zone.

        Args:
            position: Position to check

        Returns:
            True if in any active denial zone
        """
        return any(zone.contains(position) for zone in self._config.zones)

    def get_denial_zones_at(self, position: PositionNED) -> List[GPSDenialZone]:
        """Get all denial zones containing position.

        Args:
            position: Position to check

        Returns:
            List of zones containing the position
        """
        return [zone for zone in self._config.zones if zone.contains(position)]

    @property
    def jammed_drones(self) -> Set[int]:
        """Get set of currently GPS-denied drones."""
        return self._jammed_drones.copy()

    @property
    def zones(self) -> List[GPSDenialZone]:
        """Get list of denial zones."""
        return self._config.zones.copy()

    def get_stats(self) -> Dict:
        """Get GPS denial statistics.

        Returns:
            Dictionary with statistics
        """
        return {
            "total_drones": len(self._sources),
            "currently_jammed": len(self._jammed_drones),
            "jammed_drone_ids": list(self._jammed_drones),
            "total_denial_time": dict(self._total_denial_time),
            "zone_entry_counts": dict(self._zone_entry_count),
            "active_zones": sum(1 for z in self._config.zones if z.active),
            "total_zones": len(self._config.zones),
        }

    def on_denial(self, callback: Callable[[int], None]) -> None:
        """Register callback for GPS denial events.

        Args:
            callback: Function called with drone_id when GPS is denied
        """
        self._on_denial.append(callback)

    def on_restore(self, callback: Callable[[int], None]) -> None:
        """Register callback for GPS restore events.

        Args:
            callback: Function called with drone_id when GPS is restored
        """
        self._on_restore.append(callback)


def create_urban_denial_zones(
    city_center: PositionNED,
    num_buildings: int = 5,
    building_radius: float = 15.0,
    spread: float = 50.0,
    seed: Optional[int] = None,
) -> List[GPSDenialZone]:
    """Create GPS denial zones simulating urban GPS shadowing.

    Simulates tall buildings blocking GPS signals.

    Args:
        city_center: Center of urban area (NED)
        num_buildings: Number of building shadows
        building_radius: Shadow radius per building
        spread: How far buildings spread from center
        seed: Random seed for reproducibility

    Returns:
        List of GPS denial zones
    """
    rng = random.Random(seed)
    zones = []

    for i in range(num_buildings):
        # Random position around city center
        angle = rng.uniform(0, 2 * math.pi)
        dist = rng.uniform(0, spread)

        north = city_center[0] + dist * math.cos(angle)
        east = city_center[1] + dist * math.sin(angle)
        down = city_center[2]

        zones.append(GPSDenialZone(
            zone_id=100 + i,
            center=(north, east, down),
            radius=building_radius * rng.uniform(0.8, 1.2),
            shape=DenialZoneShape.CYLINDER,
            height=50.0,  # Building height
            probability=0.8,  # 80% denial (some GPS might work)
            name=f"Building_{i+1}",
        ))

    return zones


def create_canyon_denial_zone(
    start: PositionNED,
    end: PositionNED,
    width: float = 30.0,
    probability: float = 0.9,
) -> List[GPSDenialZone]:
    """Create GPS denial zones simulating a canyon.

    Args:
        start: Start of canyon (NED)
        end: End of canyon (NED)
        width: Canyon width
        probability: Denial probability

    Returns:
        List of overlapping cylindrical zones
    """
    zones = []

    # Create overlapping cylinders along canyon
    sn, se, sd = start
    en, ee, ed = end

    length = math.sqrt((en - sn)**2 + (ee - se)**2)
    num_zones = max(2, int(length / (width / 2)))

    for i in range(num_zones):
        t = i / (num_zones - 1) if num_zones > 1 else 0.5
        north = sn + t * (en - sn)
        east = se + t * (ee - se)
        down = sd + t * (ed - sd)

        zones.append(GPSDenialZone(
            zone_id=200 + i,
            center=(north, east, down),
            radius=width / 2,
            shape=DenialZoneShape.CYLINDER,
            height=100.0,  # Tall canyon walls
            probability=probability,
            name=f"Canyon_segment_{i+1}",
        ))

    return zones
