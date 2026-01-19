"""Configuration management for drone swarm."""

from dataclasses import dataclass


@dataclass
class SwarmConfig:
    """Configuration for the entire swarm.

    This is the primary configuration class for swarm operations.
    Port calculations and spawn positions are handled by the
    swarm.simulation module for simulation setup.
    """

    num_drones: int = 6
    base_port: int = 14540

    # Formation defaults
    formation_spacing: float = 5.0  # meters between drones

    # Communication
    ros_namespace: str = "swarm"

    def get_mavlink_port(self, instance_id: int) -> int:
        """Get UDP port for MAVLink connection.

        Args:
            instance_id: Drone instance number (0-indexed)

        Returns:
            UDP port (base_port + instance_id)
        """
        return self.base_port + instance_id
