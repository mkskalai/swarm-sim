"""Terrain configuration for autonomous mission environments.

Provides configuration classes for different terrain types used in
Gazebo world generation.
"""

import random
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple, Optional


class TerrainType(Enum):
    """Types of terrain environments."""
    FLAT = "flat"
    URBAN = "urban"
    FOREST = "forest"
    CANYON = "canyon"


@dataclass
class ObstacleConfig:
    """Configuration for a single obstacle.

    Attributes:
        name: Unique obstacle name
        position: (x, y, z) position in world
        dimensions: (width, depth, height) or (radius, height) for cylinders
        shape: "box", "cylinder", or "sphere"
        color: (r, g, b, a) color values 0-1
        collision: Whether obstacle has collision
    """
    name: str
    position: Tuple[float, float, float]
    dimensions: Tuple[float, ...]
    shape: str = "box"
    color: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)
    collision: bool = True


@dataclass
class GPSDenialZoneConfig:
    """Configuration for GPS denial zone in terrain.

    Attributes:
        center: (x, y, z) center position
        radius: Zone radius in meters
        probability: Denial probability (0-1)
    """
    center: Tuple[float, float, float]
    radius: float
    probability: float = 1.0


@dataclass
class TerrainConfig:
    """Configuration for terrain world generation.

    Attributes:
        terrain_type: Type of terrain
        size: (width, height) of terrain in meters
        obstacle_density: Density of obstacles (0-1)
        seed: Random seed for reproducible generation
        obstacles: List of obstacle configurations
        gps_denial_zones: List of GPS denial zone configurations
        ground_color: Ground plane color (r, g, b, a)
        description: Human-readable description
    """
    terrain_type: TerrainType
    size: Tuple[float, float] = (200.0, 200.0)
    obstacle_density: float = 0.3
    seed: int = 42
    obstacles: List[ObstacleConfig] = field(default_factory=list)
    gps_denial_zones: List[GPSDenialZoneConfig] = field(default_factory=list)
    ground_color: Tuple[float, float, float, float] = (0.3, 0.4, 0.3, 1.0)
    description: str = ""

    def generate_obstacles(self) -> None:
        """Generate obstacles based on terrain type and density."""
        rng = random.Random(self.seed)

        if self.terrain_type == TerrainType.URBAN:
            self._generate_urban_obstacles(rng)
        elif self.terrain_type == TerrainType.FOREST:
            self._generate_forest_obstacles(rng)
        elif self.terrain_type == TerrainType.CANYON:
            self._generate_canyon_obstacles(rng)

    def _generate_urban_obstacles(self, rng: random.Random) -> None:
        """Generate urban environment obstacles (buildings, vehicles)."""
        width, height = self.size

        # Grid-based building placement
        grid_size = 30.0  # Space between building centers
        margin = 20.0     # Keep drones spawn area clear

        num_x = int((width - 2 * margin) / grid_size)
        num_y = int((height - 2 * margin) / grid_size)

        building_id = 0

        for i in range(num_x):
            for j in range(num_y):
                # Skip some buildings based on density
                if rng.random() > self.obstacle_density:
                    continue

                x = margin + i * grid_size + rng.uniform(-5, 5)
                y = margin + j * grid_size + rng.uniform(-5, 5)

                # Vary building size
                bw = rng.uniform(6.0, 12.0)
                bd = rng.uniform(6.0, 12.0)
                bh = rng.uniform(8.0, 25.0)

                # Building color (gray-ish)
                gray = rng.uniform(0.3, 0.7)
                color = (gray, gray, gray * 0.9, 1.0)

                self.obstacles.append(ObstacleConfig(
                    name=f"building_{building_id}",
                    position=(x, y, bh / 2),
                    dimensions=(bw, bd, bh),
                    shape="box",
                    color=color,
                ))

                # Add GPS denial zone near tall buildings
                if bh > 15:
                    self.gps_denial_zones.append(GPSDenialZoneConfig(
                        center=(x, y, bh / 2),
                        radius=bh * 0.8,
                        probability=0.8,
                    ))

                building_id += 1

        # Add some vehicles on "streets"
        num_vehicles = int(10 * self.obstacle_density)
        for v_id in range(num_vehicles):
            x = rng.uniform(margin, width - margin)
            y = rng.uniform(margin, height - margin)

            # Check not inside a building
            in_building = False
            for obs in self.obstacles:
                if obs.shape == "box":
                    dx = abs(x - obs.position[0])
                    dy = abs(y - obs.position[1])
                    if dx < obs.dimensions[0] / 2 + 2 and dy < obs.dimensions[1] / 2 + 2:
                        in_building = True
                        break

            if not in_building:
                # Car dimensions
                self.obstacles.append(ObstacleConfig(
                    name=f"vehicle_{v_id}",
                    position=(x, y, 0.75),
                    dimensions=(4.5, 1.8, 1.5),
                    shape="box",
                    color=(rng.uniform(0.2, 0.8), rng.uniform(0.2, 0.4), rng.uniform(0.2, 0.4), 1.0),
                ))

        self.description = f"Urban terrain with {building_id} buildings and {num_vehicles} vehicles"
        self.ground_color = (0.3, 0.3, 0.3, 1.0)  # Gray asphalt

    def _generate_forest_obstacles(self, rng: random.Random) -> None:
        """Generate forest environment obstacles (trees)."""
        width, height = self.size
        margin = 15.0

        # Tree density based on obstacle_density
        tree_spacing = 8.0 / max(0.1, self.obstacle_density)
        num_x = int((width - 2 * margin) / tree_spacing)
        num_y = int((height - 2 * margin) / tree_spacing)

        tree_id = 0

        for i in range(num_x):
            for j in range(num_y):
                # Add randomness to positions
                x = margin + i * tree_spacing + rng.uniform(-3, 3)
                y = margin + j * tree_spacing + rng.uniform(-3, 3)

                # Skip some trees randomly
                if rng.random() > 0.7:
                    continue

                # Tree dimensions
                trunk_radius = rng.uniform(0.2, 0.5)
                tree_height = rng.uniform(8.0, 18.0)

                # Trunk (brown cylinder)
                self.obstacles.append(ObstacleConfig(
                    name=f"tree_trunk_{tree_id}",
                    position=(x, y, tree_height * 0.3),
                    dimensions=(trunk_radius, tree_height * 0.6),
                    shape="cylinder",
                    color=(0.4, 0.25, 0.1, 1.0),
                ))

                # Canopy (green sphere/ellipsoid approximated as cylinder)
                canopy_radius = rng.uniform(2.0, 4.0)
                self.obstacles.append(ObstacleConfig(
                    name=f"tree_canopy_{tree_id}",
                    position=(x, y, tree_height * 0.7),
                    dimensions=(canopy_radius, tree_height * 0.5),
                    shape="cylinder",
                    color=(0.1 + rng.uniform(0, 0.15), 0.4 + rng.uniform(0, 0.2), 0.1, 1.0),
                    collision=False,  # Can fly through canopy edge
                ))

                tree_id += 1

        # Create clearings (areas without trees) by removing some obstacles
        num_clearings = 3
        for _ in range(num_clearings):
            cx = rng.uniform(margin + 20, width - margin - 20)
            cy = rng.uniform(margin + 20, height - margin - 20)
            clearing_radius = rng.uniform(10, 20)

            self.obstacles = [
                obs for obs in self.obstacles
                if math.sqrt((obs.position[0] - cx)**2 + (obs.position[1] - cy)**2) > clearing_radius
            ]

        self.description = f"Forest terrain with {tree_id} trees and {num_clearings} clearings"
        self.ground_color = (0.25, 0.35, 0.2, 1.0)  # Forest green

    def _generate_canyon_obstacles(self, rng: random.Random) -> None:
        """Generate canyon environment obstacles (walls)."""
        width, height = self.size

        # Canyon runs through center
        canyon_width = rng.uniform(25, 40)
        wall_height = rng.uniform(40, 60)

        # Left wall
        num_segments = 10
        segment_length = height / num_segments

        for i in range(num_segments):
            y = i * segment_length + segment_length / 2

            # Add some waviness to walls
            wave = 5 * math.sin(i * 0.5) + rng.uniform(-3, 3)

            # Left wall
            self.obstacles.append(ObstacleConfig(
                name=f"left_wall_{i}",
                position=(width / 2 - canyon_width / 2 - 10 + wave, y, wall_height / 2),
                dimensions=(20, segment_length + 2, wall_height),
                shape="box",
                color=(0.6, 0.5, 0.4, 1.0),  # Rock color
            ))

            # Right wall
            self.obstacles.append(ObstacleConfig(
                name=f"right_wall_{i}",
                position=(width / 2 + canyon_width / 2 + 10 - wave, y, wall_height / 2),
                dimensions=(20, segment_length + 2, wall_height),
                shape="box",
                color=(0.55, 0.45, 0.35, 1.0),
            ))

        # Add some boulders in canyon floor
        num_boulders = int(8 * self.obstacle_density)
        for b_id in range(num_boulders):
            x = width / 2 + rng.uniform(-canyon_width / 3, canyon_width / 3)
            y = rng.uniform(20, height - 20)
            size = rng.uniform(1.5, 4.0)

            self.obstacles.append(ObstacleConfig(
                name=f"boulder_{b_id}",
                position=(x, y, size / 2),
                dimensions=(size, size, size),
                shape="box",
                color=(0.5, 0.45, 0.4, 1.0),
            ))

        # GPS denial throughout canyon (walls block signal)
        self.gps_denial_zones.append(GPSDenialZoneConfig(
            center=(width / 2, height / 2, wall_height / 2),
            radius=canyon_width,
            probability=0.9,
        ))

        self.description = f"Canyon terrain with {wall_height}m walls, {canyon_width}m passage"
        self.ground_color = (0.5, 0.4, 0.3, 1.0)  # Sandy/rock


def create_urban_config(
    size: Tuple[float, float] = (200.0, 200.0),
    density: float = 0.4,
    seed: int = 42,
) -> TerrainConfig:
    """Create urban terrain configuration.

    Args:
        size: Terrain size (width, height) in meters
        density: Building density (0-1)
        seed: Random seed

    Returns:
        Configured TerrainConfig
    """
    config = TerrainConfig(
        terrain_type=TerrainType.URBAN,
        size=size,
        obstacle_density=density,
        seed=seed,
    )
    config.generate_obstacles()
    return config


def create_forest_config(
    size: Tuple[float, float] = (200.0, 200.0),
    density: float = 0.5,
    seed: int = 42,
) -> TerrainConfig:
    """Create forest terrain configuration.

    Args:
        size: Terrain size (width, height) in meters
        density: Tree density (0-1)
        seed: Random seed

    Returns:
        Configured TerrainConfig
    """
    config = TerrainConfig(
        terrain_type=TerrainType.FOREST,
        size=size,
        obstacle_density=density,
        seed=seed,
    )
    config.generate_obstacles()
    return config


def create_canyon_config(
    size: Tuple[float, float] = (200.0, 200.0),
    density: float = 0.3,
    seed: int = 42,
) -> TerrainConfig:
    """Create canyon terrain configuration.

    Args:
        size: Terrain size (width, height) in meters
        density: Boulder density (0-1)
        seed: Random seed

    Returns:
        Configured TerrainConfig
    """
    config = TerrainConfig(
        terrain_type=TerrainType.CANYON,
        size=size,
        obstacle_density=density,
        seed=seed,
    )
    config.generate_obstacles()
    return config
