#!/usr/bin/env python3
"""Generate multi-drone Gazebo world files from Jinja2 template.

This script generates SDF world files for multi-drone simulations.
Each drone gets unique FDM ports for ArduPilot communication.

Usage:
    python scripts/generate_world.py --num-drones 3
    python scripts/generate_world.py --num-drones 6 --spacing 8.0 --output worlds/custom.sdf
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


def generate_world(
    num_drones: int,
    spacing: float = 5.0,
    output_path: Path | None = None,
    base_fdm_port: int = 9002,
    base_sitl_port: int = 5760,
) -> Path:
    """Generate SDF world file with N drones.

    Args:
        num_drones: Number of drones to spawn.
        spacing: Distance between drones in meters.
        output_path: Output file path. Defaults to worlds/multi_drone_N.sdf
        base_fdm_port: Base FDM port for first drone.
        base_sitl_port: Base SITL TCP port for first drone.

    Returns:
        Path to generated world file.
    """
    # Use FleetConfig for consistent port/position calculations
    config = FleetConfig(
        num_drones=num_drones,
        base_fdm_port=base_fdm_port,
        base_sitl_port=base_sitl_port,
        formation_spacing=spacing,
    )

    # Build drone configuration list
    drones = []
    for i in range(num_drones):
        x, y, z = config.get_spawn_position(i)
        drones.append({
            "name": f"drone_{i}",
            "instance_id": i,
            "fdm_port": config.get_fdm_port(i),
            "mavsdk_port": config.get_mavsdk_port(i),
            "x": x,
            "y": y,
            "z": z,
            "yaw": 0.0,
        })

    # Load and render template
    template_dir = PROJECT_ROOT / "worlds"
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template("multi_drone.sdf.jinja")

    world_sdf = template.render(
        num_drones=num_drones,
        drones=drones,
    )

    # Determine output path
    if output_path is None:
        output_path = template_dir / f"multi_drone_{num_drones}.sdf"
    else:
        output_path = Path(output_path)

    # Write output
    output_path.write_text(world_sdf)
    print(f"Generated world with {num_drones} drones: {output_path}")

    # Print port summary
    print("\nPort assignments:")
    print(f"{'Drone':<10} {'FDM Port':<12} {'MAVSDK Port':<12} {'Position'}")
    print("-" * 50)
    for drone in drones:
        pos = f"({drone['x']:.1f}, {drone['y']:.1f}, {drone['z']:.3f})"
        print(f"{drone['name']:<10} {drone['fdm_port']:<12} {drone['mavsdk_port']:<12} {pos}")

    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate multi-drone Gazebo world files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/generate_world.py --num-drones 3
    python scripts/generate_world.py --num-drones 6 --spacing 8.0
    python scripts/generate_world.py -n 3 -o worlds/test.sdf

After generating, run:
    cd ~/ardupilot_gazebo
    gz sim -r ~/swarm/worlds/multi_drone_3.sdf
        """,
    )
    parser.add_argument(
        "-n", "--num-drones",
        type=int,
        default=3,
        help="Number of drones (default: 3)",
    )
    parser.add_argument(
        "-s", "--spacing",
        type=float,
        default=5.0,
        help="Distance between drones in meters (default: 5.0)",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        help="Output file path (default: worlds/multi_drone_N.sdf)",
    )
    parser.add_argument(
        "--fdm-port",
        type=int,
        default=9002,
        help="Base FDM port (default: 9002)",
    )
    parser.add_argument(
        "--sitl-port",
        type=int,
        default=5760,
        help="Base SITL TCP port (default: 5760)",
    )

    args = parser.parse_args()

    if args.num_drones < 1:
        parser.error("Number of drones must be at least 1")

    if args.num_drones > 15:
        print(f"Warning: {args.num_drones} drones may require significant resources")

    output_path = Path(args.output) if args.output else None

    generate_world(
        num_drones=args.num_drones,
        spacing=args.spacing,
        output_path=output_path,
        base_fdm_port=args.fdm_port,
        base_sitl_port=args.sitl_port,
    )


if __name__ == "__main__":
    main()
