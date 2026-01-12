"""Phase 1 tests for single drone foundation.

These tests verify the core drone functionality without requiring
the full simulation stack (Gazebo + SITL).
"""

import pytest
from swarm.core import Drone, DroneConfig
from swarm.core.config import SwarmConfig
from swarm.core.drone import DroneState, Position, GlobalPosition


class TestDroneConfig:
    """Tests for DroneConfig."""

    def test_default_config(self):
        """Test default configuration values."""
        config = DroneConfig()
        assert config.system_address == "udp://:14540"
        assert config.instance_id == 0
        assert config.default_altitude == 10.0
        assert config.default_speed == 5.0

    def test_instance_config(self):
        """Test config generation for different instances."""
        config0 = DroneConfig.for_instance(0)
        assert config0.system_address == "udp://:14540"
        assert config0.instance_id == 0

        config1 = DroneConfig.for_instance(1)
        assert config1.system_address == "udp://:14550"
        assert config1.instance_id == 1

        config2 = DroneConfig.for_instance(2)
        assert config2.system_address == "udp://:14560"
        assert config2.instance_id == 2

    def test_custom_base_port(self):
        """Test config with custom base port."""
        config = DroneConfig.for_instance(0, base_port=15000)
        assert config.system_address == "udp://:15000"


class TestSwarmConfig:
    """Tests for SwarmConfig."""

    def test_default_swarm_config(self):
        """Test default swarm configuration."""
        config = SwarmConfig()
        assert config.num_drones == 6
        assert config.base_port == 14540

    def test_generate_drone_configs(self):
        """Test generation of drone configs for swarm."""
        config = SwarmConfig(num_drones=3)
        drone_configs = config.get_drone_configs()

        assert len(drone_configs) == 3
        assert drone_configs[0].system_address == "udp://:14540"
        assert drone_configs[1].system_address == "udp://:14550"
        assert drone_configs[2].system_address == "udp://:14560"


class TestDroneState:
    """Tests for drone state and data classes."""

    def test_drone_state_enum(self):
        """Test DroneState enum values."""
        assert DroneState.DISCONNECTED.value == "disconnected"
        assert DroneState.CONNECTED.value == "connected"
        assert DroneState.ARMED.value == "armed"
        assert DroneState.IN_FLIGHT.value == "in_flight"

    def test_position_altitude(self):
        """Test Position altitude property."""
        pos = Position(north=10.0, east=5.0, down=-15.0)
        assert pos.altitude == 15.0  # -down = altitude

    def test_global_position(self):
        """Test GlobalPosition dataclass."""
        pos = GlobalPosition(
            latitude=-35.363262,
            longitude=149.165237,
            altitude=600.0,
            yaw=45.0,
        )
        assert pos.latitude == -35.363262
        assert pos.longitude == 149.165237
        assert pos.altitude == 600.0
        assert pos.yaw == 45.0


class TestDroneInit:
    """Tests for Drone initialization (no connection required)."""

    def test_drone_default_init(self):
        """Test drone initialization with defaults."""
        drone = Drone()
        assert drone.state == DroneState.DISCONNECTED
        assert not drone.is_connected
        assert not drone.is_armed
        assert drone.position is None
        assert drone.global_position is None

    def test_drone_custom_config(self):
        """Test drone initialization with custom config."""
        config = DroneConfig.for_instance(1)
        drone = Drone(config)
        assert drone.config.instance_id == 1
        assert drone.config.system_address == "udp://:14550"

    def test_drone_repr(self):
        """Test drone string representation."""
        drone = Drone()
        repr_str = repr(drone)
        assert "Drone" in repr_str
        assert "instance=0" in repr_str
        assert "disconnected" in repr_str


class TestCameraConfig:
    """Tests for camera configuration."""

    def test_camera_config_defaults(self):
        """Test default camera configuration."""
        from swarm.perception.camera import CameraConfig

        config = CameraConfig()
        assert config.topic == "/drone/camera"
        assert config.width == 640
        assert config.height == 480
        assert config.fps == 30.0


# Integration tests (require SITL + Gazebo)
@pytest.mark.skip(reason="Requires SITL + Gazebo running")
class TestDroneIntegration:
    """Integration tests requiring full simulation stack."""

    @pytest.mark.asyncio
    async def test_connect_to_sitl(self):
        """Test connection to SITL."""
        drone = Drone()
        connected = await drone.connect(timeout=10.0)
        assert connected
        assert drone.is_connected
        await drone.disconnect()

    @pytest.mark.asyncio
    async def test_takeoff_land(self):
        """Test basic takeoff and landing."""
        drone = Drone()
        await drone.connect()

        assert await drone.takeoff(altitude=5.0)
        assert drone.state == DroneState.IN_FLIGHT

        assert await drone.land()
        assert drone.state == DroneState.LANDED

        await drone.disconnect()
