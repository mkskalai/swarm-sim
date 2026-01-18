"""Unit tests for Phase 2: Multi-Drone Spawning.

These tests verify the configuration and Fleet class initialization
without requiring actual SITL or Gazebo simulation.
"""

import os
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
        assert config.base_mavlink_port == 14540
        assert config.base_fdm_port == 9002
        assert config.formation_spacing == 5.0

    def test_mavlink_port_generation(self):
        """Test MAVLink UDP port calculation for each instance."""
        config = FleetConfig()

        # UDP ports are sequential: 14540, 14541, 14542
        assert config.get_mavlink_port(0) == 14540
        assert config.get_mavlink_port(1) == 14541
        assert config.get_mavlink_port(2) == 14542

    def test_fdm_port_generation(self):
        """Test FDM port calculation for Gazebo."""
        config = FleetConfig()

        assert config.get_fdm_port(0) == 9002
        assert config.get_fdm_port(1) == 9012
        assert config.get_fdm_port(2) == 9022

    def test_custom_base_ports(self):
        """Test custom base port configuration."""
        config = FleetConfig(
            base_mavlink_port=15000,
            base_fdm_port=10000,
        )

        # UDP ports are sequential
        assert config.get_mavlink_port(0) == 15000
        assert config.get_mavlink_port(1) == 15001
        # FDM ports use offset of 10
        assert config.get_fdm_port(0) == 10000
        assert config.get_fdm_port(1) == 10010

    def test_drone_config_generation(self):
        """Test DroneConfig generation for instances."""
        config = FleetConfig(num_drones=3)

        drone_config_0 = config.get_drone_config(0)
        drone_config_1 = config.get_drone_config(1)

        # Uses UDP connection for MAVProxy forwarding
        assert drone_config_0.system_address == "udp://:14540"
        assert drone_config_0.instance_id == 0

        assert drone_config_1.system_address == "udp://:14541"
        assert drone_config_1.instance_id == 1

    def test_all_drone_configs(self):
        """Test generation of all drone configs."""
        config = FleetConfig(num_drones=3)
        configs = config.get_all_drone_configs()

        assert len(configs) == 3

        for i, drone_config in enumerate(configs):
            expected_port = 14540 + i  # Sequential UDP ports
            assert drone_config.system_address == f"udp://:{expected_port}"
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
        assert config.get_mavlink_port(9) == 14540 + 9  # Sequential UDP
        assert config.get_fdm_port(9) == 9002 + 90  # 10x offset for FDM


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


# Check if simulation is running (set by run_integration_tests.py)
SIM_RUNNING = os.environ.get("SWARM_SIM_RUNNING", "").lower() in ("1", "true", "yes")

# Skip reason for integration tests
SKIP_REASON = "Requires running SITL and Gazebo. Use: ./scripts/run.sh sim-test"


# Integration tests that require simulation
# NOTE: Uses SwarmController (pymavlink) instead of Fleet (MAVSDK) because
# MAVSDK routes by sysid which fails for multi-drone even with unique sysids.
# pymavlink uses port isolation which works reliably.
@pytest.mark.integration
class TestFleetIntegration:
    """Integration tests requiring SITL and Gazebo.

    These tests are skipped unless simulation is running. Run with:
        python scripts/run_integration_tests.py
    Or in Docker:
        ./scripts/run.sh sim-test
    """

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_fleet_connection(self):
        """Test connecting to SITL fleet using pymavlink."""
        from swarm.coordination import SwarmController, SwarmConfig

        num_drones = int(os.environ.get("SWARM_NUM_DRONES", "3"))
        config = SwarmConfig(num_drones=num_drones)
        controller = SwarmController(config)

        try:
            success = controller.connect_all(timeout=30.0)
            assert success is True
            assert controller.is_connected is True

        finally:
            controller.disconnect_all()

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_fleet_takeoff_land(self):
        """Test fleet takeoff and landing using pymavlink."""
        from swarm.coordination import SwarmController, SwarmConfig

        num_drones = int(os.environ.get("SWARM_NUM_DRONES", "3"))
        config = SwarmConfig(num_drones=num_drones)
        controller = SwarmController(config)

        try:
            controller.connect_all()
            controller.wait_for_ekf_all()
            controller.set_mode_all('GUIDED')
            controller.arm_all()
            controller.takeoff_all(altitude=5.0)

            assert controller.is_armed is True

            controller.land_all()

        finally:
            controller.disconnect_all()
