#!/usr/bin/env python3
"""Generate terrain-based Gazebo world files for Phase 7 autonomous missions.

This script generates world files with terrain obstacles (urban, forest, canyon)
suitable for testing autonomous navigation in complex environments.

Usage:
    python scripts/generate_terrain_world.py --terrain urban --num-drones 4
    python scripts/generate_terrain_world.py --terrain forest --num-drones 6 --density 0.5
    python scripts/generate_terrain_world.py --terrain canyon --num-drones 4 --seed 123

Prerequisites:
    - ardupilot_gazebo installed (provides iris_with_standoffs model)
    - GZ_SIM_RESOURCE_PATH includes ardupilot_gazebo/models and this project's models/
"""

import argparse
import sys
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

try:
    from jinja2 import Environment, FileSystemLoader
except ImportError:
    print("Error: jinja2 is required. Install with: pip install jinja2")
    sys.exit(1)

from swarm.core.config import FleetConfig
from swarm.missions.terrain_config import (
    TerrainType,
    TerrainConfig,
    create_urban_config,
    create_forest_config,
    create_canyon_config,
)
from scripts.generate_world import ensure_drone_model

# Path to project models directory
PROJECT_MODELS = PROJECT_ROOT / "models"


def generate_terrain_world(
    terrain_type: str,
    num_drones: int,
    density: float = 0.4,
    seed: int = 42,
    size: tuple[float, float] = (200.0, 200.0),
    spacing: float = 5.0,
    output_path: Path | None = None,
    base_fdm_port: int = 9002,
    base_sitl_port: int = 5760,
) -> Path:
    """Generate SDF world file with terrain obstacles and N drones.

    Args:
        terrain_type: Type of terrain ("urban", "forest", "canyon", or "flat")
        num_drones: Number of drones to spawn
        density: Obstacle density (0-1)
        seed: Random seed for reproducible terrain
        size: Terrain size (width, height) in meters
        spacing: Distance between drones in meters
        output_path: Output file path
        base_fdm_port: Base FDM port for first drone
        base_sitl_port: Base SITL TCP port for first drone

    Returns:
        Path to generated world file
    """
    # Use FleetConfig for consistent port/position calculations
    config = FleetConfig(
        num_drones=num_drones,
        base_fdm_port=base_fdm_port,
        base_sitl_port=base_sitl_port,
        formation_spacing=spacing,
    )

    # Create terrain configuration
    terrain_type_enum = TerrainType(terrain_type)

    if terrain_type_enum == TerrainType.URBAN:
        terrain_config = create_urban_config(size=size, density=density, seed=seed)
    elif terrain_type_enum == TerrainType.FOREST:
        terrain_config = create_forest_config(size=size, density=density, seed=seed)
    elif terrain_type_enum == TerrainType.CANYON:
        terrain_config = create_canyon_config(size=size, density=density, seed=seed)
    else:
        # Flat terrain - no obstacles
        terrain_config = TerrainConfig(
            terrain_type=TerrainType.FLAT,
            size=size,
            description="Flat terrain",
        )

    # Ensure models directory exists
    PROJECT_MODELS.mkdir(exist_ok=True)

    # Ensure all drone models exist
    print(f"\nEnsuring {num_drones} drone models exist in {PROJECT_MODELS}...")
    for i in range(num_drones):
        drone_number = i + 1
        fdm_port = config.get_fdm_port(i)
        if not ensure_drone_model(drone_number, fdm_port):
            raise RuntimeError(f"Failed to create Drone{drone_number} model")

    # Build drone configuration list
    drones = []
    for i in range(num_drones):
        x, y, z = config.get_spawn_position(i)
        drone_number = i + 1
        drones.append({
            "name": f"Drone{drone_number}",
            "instance_id": i,
            "fdm_port": config.get_fdm_port(i),
            "mavsdk_port": config.get_mavsdk_port(i),
            "x": x,
            "y": y,
            "z": z,
            "yaw": 0.0,
        })

    # Convert obstacles to template format
    obstacles = []
    for obs in terrain_config.obstacles:
        obstacles.append({
            "name": obs.name,
            "position": obs.position,
            "dimensions": obs.dimensions,
            "shape": obs.shape,
            "color": obs.color,
            "collision": obs.collision,
        })

    # Load and render template
    template_dir = PROJECT_ROOT / "worlds"
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template("terrain.sdf.jinja")

    world_sdf = template.render(
        terrain_type=terrain_type,
        description=terrain_config.description,
        num_drones=num_drones,
        size=terrain_config.size,
        ground_color=terrain_config.ground_color,
        obstacles=obstacles,
        drones=drones,
    )

    # Determine output path
    if output_path is None:
        output_path = template_dir / f"terrain_{terrain_type}_{num_drones}.sdf"
    else:
        output_path = Path(output_path)

    # Write output
    output_path.write_text(world_sdf)
    print(f"\nGenerated {terrain_type} terrain world: {output_path}")
    print(f"  Terrain: {terrain_config.description}")
    print(f"  Obstacles: {len(terrain_config.obstacles)}")
    print(f"  GPS denial zones: {len(terrain_config.gps_denial_zones)}")

    # Print port summary
    print("\nPort assignments:")
    print(f"{'Drone':<10} {'FDM Port':<12} {'MAVSDK Port':<12} {'Position'}")
    print("-" * 50)
    for drone in drones:
        pos = f"({drone['x']:.1f}, {drone['y']:.1f}, {drone['z']:.3f})"
        print(f"{drone['name']:<10} {drone['fdm_port']:<12} {drone['mavsdk_port']:<12} {pos}")

    # Print GPS denial zone info
    if terrain_config.gps_denial_zones:
        print("\nGPS denial zones:")
        for i, zone in enumerate(terrain_config.gps_denial_zones):
            print(f"  Zone {i+1}: center={zone.center}, radius={zone.radius}m, prob={zone.probability}")

    return output_path


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Generate terrain-based Gazebo world files"
    )
    parser.add_argument(
        "--terrain", "-t",
        choices=["flat", "urban", "forest", "canyon"],
        default="urban",
        help="Terrain type (default: urban)"
    )
    parser.add_argument(
        "--num-drones", "-n",
        type=int,
        default=4,
        help="Number of drones (default: 4)"
    )
    parser.add_argument(
        "--density", "-d",
        type=float,
        default=0.4,
        help="Obstacle density 0-1 (default: 0.4)"
    )
    parser.add_argument(
        "--seed", "-s",
        type=int,
        default=42,
        help="Random seed (default: 42)"
    )
    parser.add_argument(
        "--size",
        type=float,
        nargs=2,
        default=[200.0, 200.0],
        metavar=("WIDTH", "HEIGHT"),
        help="Terrain size in meters (default: 200 200)"
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=5.0,
        help="Drone spacing in meters (default: 5.0)"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="Output file path (default: auto-generated)"
    )

    args = parser.parse_args()

    output_path = Path(args.output) if args.output else None

    generate_terrain_world(
        terrain_type=args.terrain,
        num_drones=args.num_drones,
        density=args.density,
        seed=args.seed,
        size=tuple(args.size),
        spacing=args.spacing,
        output_path=output_path,
    )


if __name__ == "__main__":
    main()
