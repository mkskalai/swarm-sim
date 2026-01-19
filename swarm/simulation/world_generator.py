"""Dynamic world and model generation for drone simulations.

This module provides utilities to generate Gazebo world files and drone models
for any number of drones with configurable spawn layouts.
"""

import math
import os
import re
import shutil
from pathlib import Path
from typing import Optional

try:
    from jinja2 import Environment, FileSystemLoader
except ImportError:
    raise ImportError("jinja2 is required. Install with: pip install jinja2")

# Use SWARM_PROJECT_ROOT env var if set (avoids paths with spaces)
PROJECT_ROOT = Path(os.environ.get("SWARM_PROJECT_ROOT", Path(__file__).parent.parent.parent))
MODELS_DIR = PROJECT_ROOT / "models"
WORLDS_DIR = PROJECT_ROOT / "worlds"
TEMPLATES_DIR = WORLDS_DIR / "templates"


def get_spawn_positions(
    num_drones: int,
    spacing: float = 5.0,
    layout: str = "grid",
) -> list[tuple[float, float, float]]:
    """Calculate spawn positions for N drones.

    Args:
        num_drones: Number of drones
        spacing: Meters between drones
        layout: "grid" (sqrt x sqrt) or "line"

    Returns:
        List of (x, y, z) positions
    """
    if num_drones <= 0:
        return []

    z = 0.195  # Default height for iris model on ground

    if layout == "line":
        return [(i * spacing, 0.0, z) for i in range(num_drones)]

    # Grid layout: ceil(sqrt(N)) x ceil(sqrt(N))
    cols = math.ceil(math.sqrt(num_drones))
    positions = []
    for i in range(num_drones):
        row, col = divmod(i, cols)
        positions.append((col * spacing, row * spacing, z))
    return positions


def get_fdm_port(instance_id: int, base_fdm_port: int = 9002) -> int:
    """Get FDM port for Gazebo ArduPilotPlugin.

    Args:
        instance_id: Drone instance number (0-indexed)
        base_fdm_port: Base FDM port

    Returns:
        FDM port number (9002 for instance 0, 9012 for instance 1, etc.)
    """
    return base_fdm_port + (instance_id * 10)


def get_mavlink_port(instance_id: int, base_port: int = 14540) -> int:
    """Get UDP port for MAVLink connection via MAVProxy.

    Args:
        instance_id: Drone instance number (0-indexed)
        base_port: Base UDP port

    Returns:
        UDP port (14540 for instance 0, 14541 for instance 1, etc.)
    """
    return base_port + instance_id


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
    model_dir = MODELS_DIR / model_name

    # Check if model already exists with correct port
    if model_dir.exists():
        sdf_file = model_dir / "model.sdf"
        if sdf_file.exists():
            content = sdf_file.read_text()
            if f"<fdm_port_in>{fdm_port}</fdm_port_in>" in content:
                return True
            # Port mismatch - regenerate
            print(f"Regenerating {model_name} with correct port {fdm_port}...")
            shutil.rmtree(model_dir)

    # Get template
    template_dir = MODELS_DIR / "DroneTemplate"
    if not template_dir.exists():
        print(f"ERROR: DroneTemplate not found at {template_dir}")
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
            content = content.replace('<model name="DroneTemplate">', f'<model name="{model_name}">')
            content = re.sub(
                r"<fdm_port_in>\d+</fdm_port_in>",
                f"<fdm_port_in>{fdm_port}</fdm_port_in>",
                content,
            )
            sdf_file.write_text(content)

        return True

    except Exception as e:
        print(f"ERROR: Failed to create model {model_name}: {e}")
        if model_dir.exists():
            shutil.rmtree(model_dir)
        return False


def generate_world(
    num_drones: int,
    world_type: str = "complex",
    layout: str = "grid",
    spacing: float = 5.0,
    output_path: Optional[Path] = None,
    base_fdm_port: int = 9002,
    base_mavlink_port: int = 14540,
    real_time_factor: float = 1.0,
) -> Path:
    """Generate world file with specified number of drones.

    Args:
        num_drones: Number of drone models to include
        world_type: "basic" or "complex"
        layout: "grid" or "line"
        spacing: Distance between drones in meters
        output_path: Optional output path (defaults to worlds/generated_N.sdf)
        base_fdm_port: Base FDM port for first drone
        base_mavlink_port: Base MAVLink UDP port
        real_time_factor: Simulation speed multiplier (1.0 = real-time, 2.0 = 2x speed)

    Returns:
        Path to generated world file
    """
    positions = get_spawn_positions(num_drones, spacing=spacing, layout=layout)

    # Ensure models directory exists
    MODELS_DIR.mkdir(exist_ok=True)

    # Generate DroneN models if they don't exist
    print(f"\nEnsuring {num_drones} drone models exist in {MODELS_DIR}...")
    for i in range(num_drones):
        drone_number = i + 1  # 1-indexed (Drone1, Drone2, etc.)
        fdm_port = get_fdm_port(i, base_fdm_port)
        if not ensure_drone_model(drone_number, fdm_port):
            raise RuntimeError(f"Failed to create Drone{drone_number} model")

    # Build drone configuration list
    drones = []
    for i in range(num_drones):
        x, y, z = positions[i]
        drone_number = i + 1
        drones.append({
            "name": f"Drone{drone_number}",
            "instance_id": i,
            "fdm_port": get_fdm_port(i, base_fdm_port),
            "mavlink_port": get_mavlink_port(i, base_mavlink_port),
            "x": x,
            "y": y,
            "z": z,
            "yaw": 0.0,
        })

    # Determine template to use
    template_file = f"{world_type}.sdf.jinja"
    template_path = TEMPLATES_DIR / template_file

    # Fall back to multi_drone.sdf.jinja if specific template doesn't exist
    if not template_path.exists():
        template_path = WORLDS_DIR / "multi_drone.sdf.jinja"
        if not template_path.exists():
            raise FileNotFoundError(f"No template found for world type '{world_type}'")

    # Load and render template
    env = Environment(loader=FileSystemLoader(template_path.parent))
    template = env.get_template(template_path.name)

    world_sdf = template.render(
        num_drones=num_drones,
        drones=drones,
        real_time_factor=real_time_factor,
        layout=layout,
    )

    # Determine output path
    if output_path is None:
        output_path = WORLDS_DIR / f"generated_{num_drones}.sdf"
    else:
        output_path = Path(output_path)

    # Check if world already exists (for informational message)
    world_existed = output_path.exists()

    # Write output
    output_path.write_text(world_sdf)

    if world_existed:
        print(f"\nWorld regenerated: {output_path}")
    else:
        print(f"\nWorld created: {output_path}")

    print(f"  Type: {world_type}, Layout: {layout}, Drones: {num_drones}")

    # Print port summary
    print("\nPort assignments:")
    print(f"{'Drone':<10} {'FDM Port':<12} {'MAVLink Port':<12} {'Position'}")
    print("-" * 55)
    for drone in drones:
        pos = f"({drone['x']:.1f}, {drone['y']:.1f}, {drone['z']:.3f})"
        print(f"{drone['name']:<10} {drone['fdm_port']:<12} {drone['mavlink_port']:<12} {pos}")

    return output_path


class WorldGenerator:
    """Class interface for world generation (for use by SimManager)."""

    WORLD_TYPES = ["basic", "complex"]

    @staticmethod
    def generate(
        num_drones: int,
        world_type: str = "complex",
        layout: str = "grid",
        spacing: float = 5.0,
        real_time_factor: float = 1.0,
    ) -> Path:
        """Generate world file for simulation.

        Args:
            num_drones: Number of drones
            world_type: "basic" or "complex"
            layout: "grid" or "line"
            spacing: Meters between drones
            real_time_factor: Simulation speed (1.0 = real-time, 2.0 = 2x)

        Returns:
            Path to generated world file
        """
        return generate_world(
            num_drones=num_drones,
            world_type=world_type,
            layout=layout,
            spacing=spacing,
            real_time_factor=real_time_factor,
        )

    @staticmethod
    def get_spawn_positions(
        num_drones: int,
        spacing: float = 5.0,
        layout: str = "grid",
    ) -> list[tuple[float, float, float]]:
        """Get spawn positions for drones."""
        return get_spawn_positions(num_drones, spacing, layout)
