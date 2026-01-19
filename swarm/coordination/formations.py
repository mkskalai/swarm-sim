"""Formation calculations for drone swarms.

Provides mathematical generators for various formation patterns that scale
to any number of drones. All positions are in NED (North-East-Down) frame.

Available formations:
- LINE: Drones in a straight line along the East axis
- GRID: Rectangular grid pattern (default)
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Tuple, Optional

# Position tuple: (north, east, down) in meters - NED frame
PositionNED = Tuple[float, float, float]


class FormationType(Enum):
    """Available formation types."""
    LINE = "line"
    GRID = "grid"


@dataclass
class FormationConfig:
    """Configuration for formation geometry.

    Attributes:
        spacing: Distance between adjacent drones (meters)
        altitude: Base altitude for formation (meters, positive = up)
        altitude_separation: Vertical offset between drones to prevent collision
        heading: Formation heading rotation (degrees, 0=North)
    """
    spacing: float = 5.0
    altitude: float = 10.0
    altitude_separation: float = 2.0
    heading: float = 0.0


class FormationCalculator:
    """Calculates drone positions for various formations.

    All formations are centered at origin (0, 0) in NED frame.
    Positions include altitude separation to prevent collision during transitions.

    Example:
        calc = FormationCalculator()
        positions = calc.calculate(FormationType.GRID, num_drones=6)
        for i, (n, e, d) in enumerate(positions):
            print(f"Drone {i}: N={n:.1f}, E={e:.1f}, Alt={-d:.1f}m")
    """

    def __init__(self, config: Optional[FormationConfig] = None):
        self.config = config or FormationConfig()

    def calculate(
        self,
        formation_type: FormationType,
        num_drones: int,
        config: Optional[FormationConfig] = None
    ) -> List[PositionNED]:
        """Calculate positions for N drones in specified formation.

        Args:
            formation_type: Type of formation to calculate
            num_drones: Number of drones in formation
            config: Optional override configuration

        Returns:
            List of (north, east, down) tuples, one per drone
        """
        if num_drones <= 0:
            return []

        cfg = config or self.config

        calculators = {
            FormationType.LINE: self._line_formation,
            FormationType.GRID: self._grid_formation,
        }

        positions = calculators[formation_type](num_drones, cfg)
        return self._rotate_formation(positions, cfg.heading)

    def _line_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """Line formation along East axis.

        Drones are centered around origin, spread evenly along East axis.
        All drones at same altitude (no vertical separation in line).
        """
        positions = []
        center_offset = (num_drones - 1) / 2

        for i in range(num_drones):
            east = (i - center_offset) * cfg.spacing
            down = -cfg.altitude
            positions.append((0.0, east, down))

        return positions

    def _grid_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """Rectangular grid formation (default).

        Creates a roughly square grid, centered at origin.
        Drones have slight altitude separation for safety.
        """
        cols = math.ceil(math.sqrt(num_drones))
        rows = math.ceil(num_drones / cols)

        positions = []
        idx = 0

        # Center the grid
        grid_north_offset = (rows - 1) * cfg.spacing / 2
        grid_east_offset = (cols - 1) * cfg.spacing / 2

        for row in range(rows):
            for col in range(cols):
                if idx >= num_drones:
                    break

                north = grid_north_offset - row * cfg.spacing
                east = col * cfg.spacing - grid_east_offset
                down = -(cfg.altitude + idx * cfg.altitude_separation)

                positions.append((north, east, down))
                idx += 1

        return positions

    def _rotate_formation(
        self, positions: List[PositionNED], heading: float
    ) -> List[PositionNED]:
        """Rotate formation around vertical axis.

        Args:
            positions: List of (north, east, down) positions
            heading: Rotation angle in degrees (0=North, 90=East)

        Returns:
            Rotated positions
        """
        if heading == 0.0:
            return positions

        angle_rad = math.radians(heading)
        cos_h = math.cos(angle_rad)
        sin_h = math.sin(angle_rad)

        rotated = []
        for n, e, d in positions:
            new_n = n * cos_h - e * sin_h
            new_e = n * sin_h + e * cos_h
            rotated.append((new_n, new_e, d))

        return rotated


@dataclass
class FormationTransition:
    """Manages smooth transition between formations.

    Uses cosine interpolation for smooth acceleration/deceleration.

    Example:
        transition = FormationTransition(
            start_positions=calc.calculate(FormationType.LINE, 3),
            end_positions=calc.calculate(FormationType.GRID, 3),
            duration=5.0
        )

        start_time = time.time()
        while True:
            t = time.time() - start_time
            positions = transition.get_positions_at_time(t)
            if t >= transition.duration:
                break
    """

    start_positions: List[PositionNED]
    end_positions: List[PositionNED]
    duration: float = 5.0

    def get_positions_at_time(self, t: float) -> List[PositionNED]:
        """Get interpolated positions at time t.

        Args:
            t: Time since transition start (seconds)

        Returns:
            Interpolated positions for all drones
        """
        # Clamp progress to [0, 1]
        progress = min(1.0, max(0.0, t / self.duration))

        # Cosine interpolation for smooth ease-in-out
        progress = 0.5 - 0.5 * math.cos(progress * math.pi)

        interpolated = []
        for start, end in zip(self.start_positions, self.end_positions):
            n = start[0] + (end[0] - start[0]) * progress
            e = start[1] + (end[1] - start[1]) * progress
            d = start[2] + (end[2] - start[2]) * progress
            interpolated.append((n, e, d))

        return interpolated

    def is_complete(self, t: float) -> bool:
        """Check if transition is complete."""
        return t >= self.duration


def get_formation_positions(
    formation_type: FormationType,
    num_drones: int,
    spacing: float = 5.0,
    altitude: float = 10.0,
    heading: float = 0.0,
) -> List[PositionNED]:
    """Convenience function for quick formation calculation.

    Args:
        formation_type: Type of formation
        num_drones: Number of drones
        spacing: Distance between drones (meters)
        altitude: Base altitude (meters)
        heading: Formation heading (degrees)

    Returns:
        List of (north, east, down) positions

    Example:
        positions = get_formation_positions(FormationType.LINE, 6)
    """
    config = FormationConfig(
        spacing=spacing,
        altitude=altitude,
        heading=heading,
    )
    calculator = FormationCalculator(config)
    return calculator.calculate(formation_type, num_drones)
