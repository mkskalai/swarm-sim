"""Unit tests for SwarmConfig configuration.

These tests verify port calculations, spawn positions, and
configuration generation without requiring simulation.

Run with: pytest tests/unit/test_config.py -v
"""

import pytest
import math

from swarm.core.config import SwarmConfig
from swarm.simulation import get_spawn_positions, get_fdm_port, get_mavlink_port


class TestSwarmConfig:
    """Tests for SwarmConfig class."""

    def test_default_values(self):
        """Test default configuration values."""
        config = SwarmConfig()

        assert config.num_drones == 6
        assert config.base_port == 14540
        assert config.formation_spacing == 5.0

    def test_custom_values(self):
        """Test custom configuration values."""
        config = SwarmConfig(
            num_drones=10,
            base_port=15000,
            formation_spacing=8.0,
        )

        assert config.num_drones == 10
        assert config.base_port == 15000
        assert config.formation_spacing == 8.0


class TestPortGeneration:
    """Tests for port calculation functions."""

    def test_mavlink_port_sequential(self):
        """Test MAVLink UDP ports are sequential."""
        assert get_mavlink_port(0) == 14540
        assert get_mavlink_port(1) == 14541
        assert get_mavlink_port(2) == 14542
        assert get_mavlink_port(9) == 14549

    def test_mavlink_port_custom_base(self):
        """Test MAVLink ports with custom base."""
        assert get_mavlink_port(0, base_port=15000) == 15000
        assert get_mavlink_port(1, base_port=15000) == 15001

    def test_fdm_port_offset(self):
        """Test FDM ports use 10x offset."""
        assert get_fdm_port(0) == 9002
        assert get_fdm_port(1) == 9012
        assert get_fdm_port(2) == 9022
        assert get_fdm_port(9) == 9092

    def test_fdm_port_custom_base(self):
        """Test FDM ports with custom base."""
        assert get_fdm_port(0, base_fdm_port=10000) == 10000
        assert get_fdm_port(1, base_fdm_port=10000) == 10010


class TestSpawnPositions:
    """Tests for spawn position calculations."""

    def test_line_layout(self):
        """Test line layout places drones in a row."""
        positions = get_spawn_positions(3, spacing=5.0, layout="line")

        assert len(positions) == 3
        assert positions[0] == (0.0, 0.0, 0.195)
        assert positions[1] == (5.0, 0.0, 0.195)
        assert positions[2] == (10.0, 0.0, 0.195)

    def test_line_layout_custom_spacing(self):
        """Test line layout with custom spacing."""
        positions = get_spawn_positions(3, spacing=10.0, layout="line")

        assert positions[0][0] == 0.0
        assert positions[1][0] == 10.0
        assert positions[2][0] == 20.0

    def test_grid_layout_4_drones(self):
        """Test grid layout for 4 drones creates 2x2."""
        positions = get_spawn_positions(4, spacing=5.0, layout="grid")

        assert len(positions) == 4
        # Grid should be 2x2 (ceil(sqrt(4)) = 2)
        # Row 0: (0,0), (5,0)
        # Row 1: (0,5), (5,5)
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        assert set(xs) == {0.0, 5.0}
        assert set(ys) == {0.0, 5.0}

    def test_grid_layout_6_drones(self):
        """Test grid layout for 6 drones creates 3x2."""
        positions = get_spawn_positions(6, spacing=5.0, layout="grid")

        assert len(positions) == 6
        # Grid should be ceil(sqrt(6))=3 columns
        # Positions: (0,0), (5,0), (10,0), (0,5), (5,5), (10,5)
        cols = math.ceil(math.sqrt(6))
        assert cols == 3

    def test_grid_layout_9_drones(self):
        """Test grid layout for 9 drones creates 3x3."""
        positions = get_spawn_positions(9, spacing=5.0, layout="grid")

        assert len(positions) == 9
        # Perfect 3x3 grid
        xs = set(p[0] for p in positions)
        ys = set(p[1] for p in positions)
        assert xs == {0.0, 5.0, 10.0}
        assert ys == {0.0, 5.0, 10.0}

    def test_zero_drones(self):
        """Test handling of zero drones."""
        positions = get_spawn_positions(0)
        assert positions == []

    def test_single_drone(self):
        """Test single drone spawns at origin."""
        positions = get_spawn_positions(1)
        assert len(positions) == 1
        assert positions[0] == (0.0, 0.0, 0.195)

    def test_many_drones(self):
        """Test configuration with many drones."""
        positions = get_spawn_positions(15, layout="grid")

        assert len(positions) == 15
        # Should fit in 4x4 grid (ceil(sqrt(15)) = 4)
        xs = set(p[0] for p in positions)
        ys = set(p[1] for p in positions)
        # With default spacing of 5.0, max x and y should be 15.0 (0, 5, 10, 15)
        assert max(xs) <= 15.0
        assert max(ys) <= 15.0
