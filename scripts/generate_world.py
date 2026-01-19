#!/usr/bin/env python3
"""Generate multi-drone Gazebo world files and drone models.

This script is a CLI wrapper around the swarm.simulation.world_generator module.

Usage:
    python scripts/generate_world.py --num-drones 3
    python scripts/generate_world.py --num-drones 6 --layout grid
    python scripts/generate_world.py -n 6 --spacing 8.0

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

from swarm.simulation import generate_world


def main():
    parser = argparse.ArgumentParser(
        description="Generate multi-drone Gazebo world files and models",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/generate_world.py --num-drones 3
    python scripts/generate_world.py --num-drones 6 --layout grid
    python scripts/generate_world.py -n 6 --spacing 8.0
    python scripts/generate_world.py -n 3 -o worlds/test.sdf

After generating, source the environment and run:
    source scripts/setup_env.sh
    gz sim -r worlds/generated_3.sdf
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
        "--layout",
        choices=["grid", "line"],
        default="grid",
        help="Spawn layout: 'grid' (sqrt x sqrt) or 'line' (default: grid)",
    )
    parser.add_argument(
        "--world",
        choices=["basic", "complex"],
        default="complex",
        help="World type (default: complex)",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        help="Output file path (default: worlds/generated_N.sdf)",
    )
    parser.add_argument(
        "--fdm-port",
        type=int,
        default=9002,
        help="Base FDM port (default: 9002)",
    )
    parser.add_argument(
        "--mavlink-port",
        type=int,
        default=14540,
        help="Base MAVLink UDP port (default: 14540)",
    )

    args = parser.parse_args()

    if args.num_drones < 1:
        parser.error("Number of drones must be at least 1")

    if args.num_drones > 15:
        print(f"Warning: {args.num_drones} drones may require significant resources")

    output_path = Path(args.output) if args.output else None

    try:
        generate_world(
            num_drones=args.num_drones,
            world_type=args.world,
            layout=args.layout,
            spacing=args.spacing,
            output_path=output_path,
            base_fdm_port=args.fdm_port,
            base_mavlink_port=args.mavlink_port,
        )
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
