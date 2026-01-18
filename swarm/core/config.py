"""Configuration management for drone swarm."""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class DroneConfig:
    """Configuration for a single drone connection."""

    # Connection settings
    # Default: UDP port forwarded by MAVProxy (14540 for instance 0)
    # udp://:port = MAVSDK listens for incoming UDP from MAVProxy
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

        Uses UDP ports forwarded by MAVProxy: 14540 + instance_id
        Instance 0: 14540, Instance 1: 14541, Instance 2: 14542, etc.
        """
        port = base_port + instance_id
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


@dataclass
class FleetConfig:
    """Configuration for drone fleet management.

    Handles port assignment for both MAVSDK connections and Gazebo FDM ports.

    Port schemes:
    - MAVLink UDP: base_mavlink_port + instance_id (14540, 14541, 14542, ...)
    - FDM ports: base_fdm_port + (instance_id * 10) (9002, 9012, 9022, ...)
    """

    num_drones: int = 3
    base_mavlink_port: int = 14540  # UDP port forwarded by MAVProxy
    base_fdm_port: int = 9002       # Gazebo ArduPilotPlugin FDM port
    formation_spacing: float = 5.0  # meters between drones

    # Legacy alias for backwards compatibility
    @property
    def base_sitl_port(self) -> int:
        return self.base_mavlink_port

    def get_drone_config(self, instance_id: int) -> DroneConfig:
        """Generate DroneConfig for a specific instance.

        Args:
            instance_id: The drone instance number (0-indexed).

        Returns:
            DroneConfig with correct UDP port.
        """
        return DroneConfig.for_instance(instance_id, self.base_mavlink_port)

    def get_all_drone_configs(self) -> list[DroneConfig]:
        """Generate DroneConfigs for all drones in the fleet.

        Returns:
            List of DroneConfig objects for each drone.
        """
        return [self.get_drone_config(i) for i in range(self.num_drones)]

    def get_fdm_port(self, instance_id: int) -> int:
        """Get FDM port for Gazebo ArduPilotPlugin.

        This is the port used for JSON SITL communication between
        Gazebo and ArduPilot SITL.

        Args:
            instance_id: The drone instance number (0-indexed).

        Returns:
            FDM port number (9002 for instance 0, 9012 for instance 1, etc.)
        """
        return self.base_fdm_port + (instance_id * 10)

    def get_mavlink_port(self, instance_id: int) -> int:
        """Get UDP port for MAVLink connection via MAVProxy.

        Args:
            instance_id: The drone instance number (0-indexed).

        Returns:
            UDP port (14540 for instance 0, 14541 for instance 1, etc.)
        """
        return self.base_mavlink_port + instance_id

    # Legacy alias
    def get_mavsdk_port(self, instance_id: int) -> int:
        """Alias for get_mavlink_port for backwards compatibility."""
        return self.get_mavlink_port(instance_id)

    def get_spawn_position(self, instance_id: int) -> tuple[float, float, float]:
        """Get spawn position for a drone in the world.

        Drones are spawned in a line along the X axis.

        Args:
            instance_id: The drone instance number (0-indexed).

        Returns:
            Tuple of (x, y, z) coordinates in meters.
        """
        x = instance_id * self.formation_spacing
        y = 0.0
        z = 0.195  # Default height for iris model on ground
        return (x, y, z)

    def get_all_spawn_positions(self) -> list[tuple[float, float, float]]:
        """Get spawn positions for all drones.

        Returns:
            List of (x, y, z) tuples for each drone.
        """
        return [self.get_spawn_position(i) for i in range(self.num_drones)]
