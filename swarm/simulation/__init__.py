"""Simulation infrastructure for drone swarm testing.

This module provides tools for managing the full simulation stack:
- Gazebo world generation and spawning
- ArduPilot SITL instance management
- ROS2 node orchestration
"""

from .sim_manager import SimManager
from .sitl_launcher import SITLLauncher, SITLProcess
from .world_generator import (
    WorldGenerator,
    generate_world,
    get_spawn_positions,
    get_fdm_port,
    get_mavlink_port,
    ensure_drone_model,
)

__all__ = [
    "SimManager",
    "SITLLauncher",
    "SITLProcess",
    "WorldGenerator",
    "generate_world",
    "get_spawn_positions",
    "get_fdm_port",
    "get_mavlink_port",
    "ensure_drone_model",
]
