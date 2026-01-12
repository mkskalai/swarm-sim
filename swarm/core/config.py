"""Configuration management for drone swarm."""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class DroneConfig:
    """Configuration for a single drone connection."""

    # Connection settings
    system_address: str = "udp://:14540"
    instance_id: int = 0

    # Timeouts (seconds)
    connection_timeout: float = 30.0
    action_timeout: float = 10.0

    # Flight parameters
    default_altitude: float = 10.0  # meters
    default_speed: float = 5.0  # m/s

    # Offboard control rates
    position_update_rate: float = 20.0  # Hz
    velocity_update_rate: float = 20.0  # Hz

    @classmethod
    def for_instance(cls, instance_id: int, base_port: int = 14540) -> "DroneConfig":
        """Create config for a specific SITL instance.

        ArduPilot SITL uses port offsets of 10 per instance.
        Instance 0: 14540, Instance 1: 14550, etc.
        """
        port = base_port + (instance_id * 10)
        return cls(
            system_address=f"udp://:{port}",
            instance_id=instance_id,
        )


@dataclass
class SwarmConfig:
    """Configuration for the entire swarm."""

    num_drones: int = 6
    base_port: int = 14540

    # Formation defaults
    formation_spacing: float = 5.0  # meters between drones

    # Communication
    ros_namespace: str = "swarm"

    def get_drone_configs(self) -> list[DroneConfig]:
        """Generate configs for all drones in the swarm."""
        return [
            DroneConfig.for_instance(i, self.base_port)
            for i in range(self.num_drones)
        ]
