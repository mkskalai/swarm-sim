"""Unit tests for Phase 2: Multi-Drone Spawning.

These tests verify the configuration and Fleet class initialization
without requiring actual SITL or Gazebo simulation.
"""

import pytest

from swarm.core import (
    DroneConfig,
    FleetConfig,
    Fleet,
    FleetState,
    FleetStatus,
    Position,
)


class TestFleetConfig:
    """Tests for FleetConfig class."""

    def test_default_values(self):
        """Test default configuration values."""
        config = FleetConfig()

        assert config.num_drones == 3
        assert config.base_sitl_port == 5760
        assert config.base_fdm_port == 9002
        assert config.port_offset == 10
        assert config.formation_spacing == 5.0

    def test_mavsdk_port_generation(self):
        """Test SITL TCP port calculation for each instance."""
        config = FleetConfig()

        assert config.get_mavsdk_port(0) == 5760
        assert config.get_mavsdk_port(1) == 5770
        assert config.get_mavsdk_port(2) == 5780

    def test_fdm_port_generation(self):
        """Test FDM port calculation for Gazebo."""
        config = FleetConfig()

        assert config.get_fdm_port(0) == 9002
        assert config.get_fdm_port(1) == 9012
        assert config.get_fdm_port(2) == 9022

    def test_custom_base_ports(self):
        """Test custom base port configuration."""
        config = FleetConfig(
            base_sitl_port=6000,
            base_fdm_port=10000,
        )

        assert config.get_mavsdk_port(0) == 6000
        assert config.get_mavsdk_port(1) == 6010
        assert config.get_fdm_port(0) == 10000
        assert config.get_fdm_port(1) == 10010

    def test_drone_config_generation(self):
        """Test DroneConfig generation for instances."""
        config = FleetConfig(num_drones=3)

        drone_config_0 = config.get_drone_config(0)
        drone_config_1 = config.get_drone_config(1)

        assert drone_config_0.system_address == "tcpout://127.0.0.1:5760"
        assert drone_config_0.instance_id == 0

        assert drone_config_1.system_address == "tcpout://127.0.0.1:5770"
        assert drone_config_1.instance_id == 1

    def test_all_drone_configs(self):
        """Test generation of all drone configs."""
        config = FleetConfig(num_drones=3)
        configs = config.get_all_drone_configs()

        assert len(configs) == 3

        for i, drone_config in enumerate(configs):
            expected_port = 5760 + (i * 10)
            assert drone_config.system_address == f"tcpout://127.0.0.1:{expected_port}"
            assert drone_config.instance_id == i

    def test_spawn_position_single(self):
        """Test spawn position for single drone."""
        config = FleetConfig(formation_spacing=5.0)

        pos = config.get_spawn_position(0)
        assert pos == (0.0, 0.0, 0.195)

        pos = config.get_spawn_position(1)
        assert pos == (5.0, 0.0, 0.195)

        pos = config.get_spawn_position(2)
        assert pos == (10.0, 0.0, 0.195)

    def test_spawn_positions_custom_spacing(self):
        """Test spawn positions with custom spacing."""
        config = FleetConfig(num_drones=3, formation_spacing=10.0)

        positions = config.get_all_spawn_positions()

        assert len(positions) == 3
        assert positions[0] == (0.0, 0.0, 0.195)
        assert positions[1] == (10.0, 0.0, 0.195)
        assert positions[2] == (20.0, 0.0, 0.195)

    def test_many_drones(self):
        """Test configuration with many drones."""
        config = FleetConfig(num_drones=10)

        assert len(config.get_all_drone_configs()) == 10
        assert len(config.get_all_spawn_positions()) == 10

        # Check last drone ports
        assert config.get_mavsdk_port(9) == 5760 + 90
        assert config.get_fdm_port(9) == 9002 + 90


class TestFleetStatus:
    """Tests for FleetStatus dataclass."""

    def test_all_connected(self):
        """Test all_connected property."""
        status = FleetStatus(total=3, connected=3, armed=0, in_flight=0)
        assert status.all_connected is True

        status = FleetStatus(total=3, connected=2, armed=0, in_flight=0)
        assert status.all_connected is False

    def test_all_armed(self):
        """Test all_armed property."""
        status = FleetStatus(total=3, connected=3, armed=3, in_flight=0)
        assert status.all_armed is True

        status = FleetStatus(total=3, connected=3, armed=2, in_flight=0)
        assert status.all_armed is False

    def test_all_in_flight(self):
        """Test all_in_flight property."""
        status = FleetStatus(total=3, connected=3, armed=3, in_flight=3)
        assert status.all_in_flight is True

        status = FleetStatus(total=3, connected=3, armed=3, in_flight=2)
        assert status.all_in_flight is False

    def test_has_errors(self):
        """Test has_errors property."""
        status = FleetStatus(total=3, connected=3, armed=0, in_flight=0)
        assert status.has_errors is False

        status = FleetStatus(
            total=3,
            connected=2,
            armed=0,
            in_flight=0,
            errors=["Drone 2: disconnected"],
        )
        assert status.has_errors is True


class TestFleet:
    """Tests for Fleet class (without simulation)."""

    def test_initialization(self):
        """Test Fleet initialization."""
        config = FleetConfig(num_drones=3)
        fleet = Fleet(config)

        assert len(fleet) == 3
        assert fleet.num_drones == 3
        assert fleet.state == FleetState.UNINITIALIZED

    def test_default_config(self):
        """Test Fleet with default configuration."""
        fleet = Fleet()

        assert len(fleet) == 3  # Default num_drones
        assert fleet.state == FleetState.UNINITIALIZED

    def test_drone_access_by_index(self):
        """Test accessing drones by index."""
        config = FleetConfig(num_drones=3)
        fleet = Fleet(config)

        drone_0 = fleet[0]
        drone_1 = fleet[1]
        drone_2 = fleet[2]

        assert drone_0.config.instance_id == 0
        assert drone_1.config.instance_id == 1
        assert drone_2.config.instance_id == 2

    def test_drone_iteration(self):
        """Test iterating over fleet."""
        config = FleetConfig(num_drones=3)
        fleet = Fleet(config)

        drone_ids = [drone.config.instance_id for drone in fleet]

        assert drone_ids == [0, 1, 2]

    def test_initial_status(self):
        """Test initial fleet status."""
        config = FleetConfig(num_drones=3)
        fleet = Fleet(config)

        status = fleet.get_status()

        assert status.total == 3
        assert status.connected == 0
        assert status.armed == 0
        assert status.in_flight == 0
        assert status.has_errors is False

    def test_repr(self):
        """Test string representation."""
        fleet = Fleet(FleetConfig(num_drones=3))

        repr_str = repr(fleet)

        assert "Fleet" in repr_str
        assert "num_drones=3" in repr_str
        assert "uninitialized" in repr_str


class TestPosition:
    """Tests for Position dataclass."""

    def test_position_creation(self):
        """Test creating Position objects."""
        pos = Position(north=10.0, east=5.0, down=-15.0, yaw=90.0)

        assert pos.north == 10.0
        assert pos.east == 5.0
        assert pos.down == -15.0
        assert pos.yaw == 90.0

    def test_altitude_property(self):
        """Test altitude calculation from down coordinate."""
        pos = Position(north=0.0, east=0.0, down=-10.0)
        assert pos.altitude == 10.0

        pos = Position(north=0.0, east=0.0, down=-25.5)
        assert pos.altitude == 25.5

    def test_default_yaw(self):
        """Test default yaw value."""
        pos = Position(north=0.0, east=0.0, down=0.0)
        assert pos.yaw == 0.0


# Integration tests that require simulation
class TestFleetIntegration:
    """Integration tests requiring SITL and Gazebo.

    These tests are skipped by default. Run with:
        pytest tests/test_phase2.py -v -m integration
    """

    @pytest.mark.skip(reason="Requires running SITL and Gazebo")
    @pytest.mark.integration
    @pytest.mark.asyncio
    async def test_fleet_connection(self):
        """Test connecting to SITL fleet."""
        config = FleetConfig(num_drones=3)
        fleet = Fleet(config)

        try:
            success = await fleet.connect_all(timeout=30.0)
            assert success is True
            assert fleet.state == FleetState.READY

            status = fleet.get_status()
            assert status.all_connected is True

        finally:
            await fleet.disconnect_all()

    @pytest.mark.skip(reason="Requires running SITL and Gazebo")
    @pytest.mark.integration
    @pytest.mark.asyncio
    async def test_fleet_takeoff_land(self):
        """Test fleet takeoff and landing."""
        config = FleetConfig(num_drones=3)
        fleet = Fleet(config)

        try:
            await fleet.connect_all()
            await fleet.arm_all()
            await fleet.takeoff_all(altitude=5.0)

            assert fleet.state == FleetState.IN_FLIGHT

            await fleet.land_all()

            assert fleet.state == FleetState.READY

        finally:
            await fleet.disconnect_all()
