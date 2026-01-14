#!/usr/bin/env python3
"""Generate multi-drone Gazebo world files and drone models.

This script:
1. Creates drone models (Drone1, Drone2, ...) in the project's models/ directory
2. Generates SDF world files for multi-drone simulations
3. Each drone gets unique FDM ports for ArduPilot communication

Models are generated from the DroneTemplate in models/DroneTemplate/.
The template includes iris_with_standoffs from ardupilot_gazebo.

Usage:
    python scripts/generate_world.py --num-drones 3
    python scripts/generate_world.py --num-drones 6 --spacing 8.0

Prerequisites:
    - ardupilot_gazebo installed (provides iris_with_standoffs model)
    - GZ_SIM_RESOURCE_PATH includes ardupilot_gazebo/models and this project's models/
"""

import argparse
import re
import shutil
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

# Path to project models directory
PROJECT_MODELS = PROJECT_ROOT / "models"


def ensure_drone_model(drone_number: int, fdm_port: int) -> bool:
    """Ensure a drone model exists in the project's models/ directory.

    Creates DroneN by copying DroneTemplate and updating the model name and fdm_port_in.

    Args:
        drone_number: Drone number (1-indexed, e.g., 1, 2, 3...)
        fdm_port: FDM port for this drone

    Returns:
        True if model exists or was created successfully
    """
    model_name = f"Drone{drone_number}"
    model_dir = PROJECT_MODELS / model_name

    # Check if model already exists with correct port
    if model_dir.exists():
        # Verify the port is correct
        sdf_file = model_dir / "model.sdf"
        if sdf_file.exists():
            content = sdf_file.read_text()
            if f"<fdm_port_in>{fdm_port}</fdm_port_in>" in content:
                return True
            # Port mismatch - regenerate
            print(f"Regenerating {model_name} with correct port {fdm_port}...")
            shutil.rmtree(model_dir)

    # Get template
    template_dir = PROJECT_MODELS / "DroneTemplate"
    if not template_dir.exists():
        print(f"ERROR: DroneTemplate not found at {template_dir}")
        print("Please ensure the project is properly set up with models/DroneTemplate/")
        return False

    print(f"Creating model {model_name} with fdm_port_in={fdm_port}...")

    try:
        # Copy the entire directory
        shutil.copytree(template_dir, model_dir)

        # Update model.config
        config_file = model_dir / "model.config"
        if config_file.exists():
            content = config_file.read_text()
            content = content.replace("<name>DroneTemplate</name>", f"<name>{model_name}</name>")
            content = content.replace("DroneTemplate", model_name)
            config_file.write_text(content)

        # Update model.sdf - change model name and fdm_port_in
        sdf_file = model_dir / "model.sdf"
        if sdf_file.exists():
            content = sdf_file.read_text()
            # Update model name
            content = content.replace('<model name="DroneTemplate">', f'<model name="{model_name}">')
            # Update fdm_port_in
            content = re.sub(
                r"<fdm_port_in>\d+</fdm_port_in>",
                f"<fdm_port_in>{fdm_port}</fdm_port_in>",
                content,
            )
            sdf_file.write_text(content)

        print(f"  Created {model_dir}")
        return True

    except Exception as e:
        print(f"ERROR: Failed to create model {model_name}: {e}")
        # Clean up partial copy
        if model_dir.exists():
            shutil.rmtree(model_dir)
        return False


def generate_world(
    num_drones: int,
    spacing: float = 5.0,
    output_path: Path | None = None,
    base_fdm_port: int = 9002,
    base_sitl_port: int = 5760,
) -> Path:
    """Generate SDF world file with N drones.

    Also creates any missing drone models in models/.

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

    # Ensure models directory exists
    PROJECT_MODELS.mkdir(exist_ok=True)

    # Ensure all drone models exist
    print(f"\nEnsuring {num_drones} drone models exist in {PROJECT_MODELS}...")
    for i in range(num_drones):
        drone_number = i + 1  # 1-indexed (Drone1, Drone2, etc.)
        fdm_port = config.get_fdm_port(i)
        if not ensure_drone_model(drone_number, fdm_port):
            raise RuntimeError(f"Failed to create Drone{drone_number} model")

    # Build drone configuration list
    # Use naming convention: Drone1, Drone2, etc. (1-indexed)
    drones = []
    for i in range(num_drones):
        x, y, z = config.get_spawn_position(i)
        drone_number = i + 1
        drones.append({
            "name": f"Drone{drone_number}",  # Match model naming
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
    print(f"\nGenerated world with {num_drones} drones: {output_path}")

    # Print port summary
    print("\nPort assignments:")
    print(f"{'Drone':<10} {'FDM Port':<12} {'MAVSDK Port':<12} {'Position'}")
    print("-" * 50)
    for drone in drones:
        pos = f"({drone['x']:.1f}, {drone['y']:.1f}, {drone['z']:.3f})"
        print(f"{drone['name']:<10} {drone['fdm_port']:<12} {drone['mavsdk_port']:<12} {pos}")

    print("\n" + "=" * 60)
    print("IMPORTANT: Ensure GZ_SIM_RESOURCE_PATH includes:")
    print(f"  - {PROJECT_MODELS} (for Drone1, Drone2, etc.)")
    print("  - ~/ardupilot_gazebo/models (for iris_with_standoffs)")
    print("=" * 60)

    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate multi-drone Gazebo world files and models",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/generate_world.py --num-drones 3
    python scripts/generate_world.py --num-drones 6 --spacing 8.0
    python scripts/generate_world.py -n 3 -o worlds/test.sdf

After generating, source the environment and run:
    source scripts/setup_env.sh
    gz sim -r worlds/multi_drone_3.sdf
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
