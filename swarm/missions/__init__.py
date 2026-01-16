"""Autonomous mission planning and execution.

This module provides:
- Terrain configuration for different environment types
- Autonomous mission orchestration
"""

from .terrain_config import (
    TerrainType,
    TerrainConfig,
    ObstacleConfig,
    create_urban_config,
    create_forest_config,
    create_canyon_config,
)

from .autonomous_mission import (
    MissionPhase,
    MissionState,
    AutonomousMissionConfig,
    AutonomousMissionController,
)

__all__ = [
    # Terrain
    "TerrainType",
    "TerrainConfig",
    "ObstacleConfig",
    "create_urban_config",
    "create_forest_config",
    "create_canyon_config",
    # Mission
    "MissionPhase",
    "MissionState",
    "AutonomousMissionConfig",
    "AutonomousMissionController",
]
