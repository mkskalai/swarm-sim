"""Formation calculations for drone swarms.

Provides mathematical generators for various formation patterns that scale
to any number of drones. All positions are in NED (North-East-Down) frame.
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
    V_FORMATION = "v"
    TRIANGLE = "triangle"
    GRID = "grid"
    CIRCLE = "circle"
    DIAMOND = "diamond"


@dataclass
class FormationConfig:
    """Configuration for formation geometry.

    Attributes:
        spacing: Distance between adjacent drones (meters)
        altitude: Base altitude for formation (meters, positive = up)
        altitude_separation: Vertical offset between drones to prevent collision
        heading: Formation heading rotation (degrees, 0=North)
        v_angle: Angle for V formation wings (degrees from centerline)
    """
    spacing: float = 5.0
    altitude: float = 10.0
    altitude_separation: float = 2.0
    heading: float = 0.0
    v_angle: float = 45.0


class FormationCalculator:
    """Calculates drone positions for various formations.

    All formations are centered at origin (0, 0) in NED frame.
    Positions include altitude separation to prevent collision during transitions.

    Example:
        calc = FormationCalculator()
        positions = calc.calculate(FormationType.V_FORMATION, num_drones=6)
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
            FormationType.V_FORMATION: self._v_formation,
            FormationType.TRIANGLE: self._triangle_formation,
            FormationType.GRID: self._grid_formation,
            FormationType.CIRCLE: self._circle_formation,
            FormationType.DIAMOND: self._diamond_formation,
        }

        positions = calculators[formation_type](num_drones, cfg)
        return self._rotate_formation(positions, cfg.heading)

    def _line_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """Line formation along East axis.

        Drones are centered around origin, spread evenly along East axis.
        """
        positions = []
        center_offset = (num_drones - 1) / 2

        for i in range(num_drones):
            east = (i - center_offset) * cfg.spacing
            down = -(cfg.altitude + i * cfg.altitude_separation)
            positions.append((0.0, east, down))

        return positions

    def _v_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """V formation (like flying geese).

        Drone 0 at apex (front), others alternate left/right on wings.
        """
        positions = [(0.0, 0.0, -cfg.altitude)]  # Leader at apex

        if num_drones == 1:
            return positions

        angle_rad = math.radians(cfg.v_angle)

        for i in range(1, num_drones):
            wing = (i + 1) // 2  # 1, 1, 2, 2, 3, 3, ...
            side = 1 if i % 2 == 1 else -1  # Alternate right/left

            # Drones trail behind leader (negative north)
            north = -wing * cfg.spacing * math.cos(angle_rad)
            east = side * wing * cfg.spacing * math.sin(angle_rad)
            down = -(cfg.altitude + i * cfg.altitude_separation)

            positions.append((north, east, down))

        return positions

    def _triangle_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """Triangular formation (equilateral, pointing North).

        Builds rows from front to back. Row 0 has 1 drone (apex),
        row 1 has 2 drones, row 2 has 3, etc.
        """
        positions = []
        row = 0
        idx = 0

        while idx < num_drones:
            drones_in_row = row + 1
            row_width = (drones_in_row - 1) * cfg.spacing
            row_start_east = -row_width / 2
            row_north = -row * cfg.spacing * math.sqrt(3) / 2

            for col in range(drones_in_row):
                if idx >= num_drones:
                    break
                east = row_start_east + col * cfg.spacing
                down = -(cfg.altitude + idx * cfg.altitude_separation)
                positions.append((row_north, east, down))
                idx += 1

            row += 1

        return positions

    def _grid_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """Rectangular grid formation.

        Creates a roughly square grid, centered at origin.
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

    def _circle_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """Circle formation.

        Drones evenly spaced around a circle. Radius calculated to maintain
        spacing between adjacent drones.
        """
        if num_drones == 1:
            return [(0.0, 0.0, -cfg.altitude)]

        # Calculate radius to achieve desired spacing between adjacent drones
        # Arc length = spacing, so: 2*pi*r / n = spacing -> r = spacing * n / (2*pi)
        radius = cfg.spacing * num_drones / (2 * math.pi)
        positions = []

        for i in range(num_drones):
            angle = 2 * math.pi * i / num_drones
            north = radius * math.cos(angle)
            east = radius * math.sin(angle)
            down = -(cfg.altitude + i * cfg.altitude_separation)
            positions.append((north, east, down))

        return positions

    def _diamond_formation(
        self, num_drones: int, cfg: FormationConfig
    ) -> List[PositionNED]:
        """Diamond formation (rhombus shape).

        4-point diamond with apex front, wings at sides, tail at rear.
        Additional drones extend the wings.
        """
        if num_drones < 4:
            return self._v_formation(num_drones, cfg)

        positions = []

        # Apex (front)
        positions.append((cfg.spacing, 0.0, -cfg.altitude))

        # Left wing
        positions.append((0.0, -cfg.spacing, -(cfg.altitude + cfg.altitude_separation)))

        # Right wing
        positions.append((0.0, cfg.spacing, -(cfg.altitude + 2 * cfg.altitude_separation)))

        # Tail
        positions.append((-cfg.spacing, 0.0, -(cfg.altitude + 3 * cfg.altitude_separation)))

        # Additional drones extend the wings outward
        for i in range(4, num_drones):
            wing_pos = (i - 4) // 2 + 2  # 2, 2, 3, 3, 4, 4, ...
            side = -1 if i % 2 == 0 else 1  # Alternate left/right

            north = -wing_pos * cfg.spacing / 3
            east = side * wing_pos * cfg.spacing
            down = -(cfg.altitude + i * cfg.altitude_separation)
            positions.append((north, east, down))

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
            end_positions=calc.calculate(FormationType.V_FORMATION, 3),
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
        positions = get_formation_positions(FormationType.V_FORMATION, 6)
    """
    config = FormationConfig(
        spacing=spacing,
        altitude=altitude,
        heading=heading,
    )
    calculator = FormationCalculator(config)
    return calculator.calculate(formation_type, num_drones)
