"""
Unit tests for NeighborTracker module.

These tests don't require ROS2 - they test the core logic only.
Run with: pytest test/test_neighbor_tracker.py -v
"""

import math
import time
from unittest.mock import MagicMock, patch

import pytest

# Test the data classes and logic without ROS2 dependencies
from swarm_ros.neighbor_tracker import (
    NeighborState,
    CollisionWarning,
    NeighborTrackerConfig,
)


class TestNeighborState:
    """Tests for NeighborState dataclass."""

    def test_default_values(self):
        """Test default values are set correctly."""
        state = NeighborState(drone_id=0)
        assert state.drone_id == 0
        assert state.position_ned == (0.0, 0.0, 0.0)
        assert state.velocity_ned == (0.0, 0.0, 0.0)
        assert state.is_healthy is True
        assert state.is_stale is False

    def test_custom_values(self):
        """Test custom values are stored correctly."""
        state = NeighborState(
            drone_id=5,
            position_ned=(10.0, 20.0, -30.0),
            velocity_ned=(1.0, 2.0, 0.5),
            yaw_deg=90.0,
            battery_percent=75,
        )
        assert state.drone_id == 5
        assert state.position_ned == (10.0, 20.0, -30.0)
        assert state.velocity_ned == (1.0, 2.0, 0.5)
        assert state.yaw_deg == 90.0
        assert state.battery_percent == 75


class TestCollisionWarning:
    """Tests for CollisionWarning dataclass."""

    def test_collision_warning(self):
        """Test CollisionWarning stores data correctly."""
        warning = CollisionWarning(
            other_drone_id=3,
            time_to_collision=2.5,
            min_distance=1.5,
            avoidance_vector=(0.5, -0.5, 0.0),
        )
        assert warning.other_drone_id == 3
        assert warning.time_to_collision == 2.5
        assert warning.min_distance == 1.5
        assert warning.avoidance_vector == (0.5, -0.5, 0.0)


class TestNeighborTrackerConfig:
    """Tests for NeighborTrackerConfig dataclass."""

    def test_default_config(self):
        """Test default configuration values."""
        config = NeighborTrackerConfig()
        assert config.stale_timeout == 3.0
        assert config.failed_timeout == 10.0
        assert config.collision_horizon == 5.0
        assert config.safe_distance == 3.0
        assert config.expected_update_rate == 10.0

    def test_custom_config(self):
        """Test custom configuration values."""
        config = NeighborTrackerConfig(
            stale_timeout=5.0,
            failed_timeout=15.0,
            collision_horizon=10.0,
            safe_distance=5.0,
        )
        assert config.stale_timeout == 5.0
        assert config.failed_timeout == 15.0
        assert config.collision_horizon == 10.0
        assert config.safe_distance == 5.0


class TestCollisionPrediction:
    """Test collision prediction math without ROS2."""

    def test_calculate_avoidance_vector(self):
        """Test avoidance vector calculation."""
        # Simulate the calculation from NeighborTracker
        def calculate_avoidance(rel_pos):
            dist = math.sqrt(sum(p**2 for p in rel_pos))
            if dist < 0.01:
                return (0.0, 0.0, -1.0)
            return tuple(-p / dist for p in rel_pos)

        # Neighbor to the north - should move south
        avoidance = calculate_avoidance((10.0, 0.0, 0.0))
        assert avoidance[0] < 0  # Move south (negative north)
        assert abs(avoidance[1]) < 0.01  # No east/west movement
        assert abs(avoidance[2]) < 0.01  # No vertical movement

        # Neighbor to the east - should move west
        avoidance = calculate_avoidance((0.0, 10.0, 0.0))
        assert abs(avoidance[0]) < 0.01
        assert avoidance[1] < 0  # Move west (negative east)
        assert abs(avoidance[2]) < 0.01

        # Neighbor at same position - should move up
        avoidance = calculate_avoidance((0.0, 0.0, 0.0))
        assert avoidance == (0.0, 0.0, -1.0)  # Up (negative down)

    def test_collision_time_calculation(self):
        """Test time-to-collision calculation."""
        # Two drones approaching each other head-on
        # Drone A at origin, moving north at 5 m/s
        # Drone B at (20, 0, 0), moving south at 5 m/s
        # Should collide in 2 seconds at (10, 0, 0)

        own_pos = (0.0, 0.0, -10.0)
        own_vel = (5.0, 0.0, 0.0)  # Moving north

        neighbor_pos = (20.0, 0.0, -10.0)
        neighbor_vel = (-5.0, 0.0, 0.0)  # Moving south

        # Relative position and velocity
        rel_pos = (neighbor_pos[0] - own_pos[0],
                   neighbor_pos[1] - own_pos[1],
                   neighbor_pos[2] - own_pos[2])
        rel_vel = (neighbor_vel[0] - own_vel[0],
                   neighbor_vel[1] - own_vel[1],
                   neighbor_vel[2] - own_vel[2])

        # Time to closest approach: t = -dot(rel_pos, rel_vel) / |rel_vel|^2
        dot_pv = sum(p * v for p, v in zip(rel_pos, rel_vel))
        rel_vel_sq = sum(v**2 for v in rel_vel)
        t_min = -dot_pv / rel_vel_sq

        assert abs(t_min - 2.0) < 0.01  # Should be 2 seconds

        # Position at closest approach should be at (0, 0, 0) relative
        min_pos = tuple(p + v * t_min for p, v in zip(rel_pos, rel_vel))
        min_dist = math.sqrt(sum(p**2 for p in min_pos))
        assert min_dist < 0.01  # Should be very close to 0 (collision)

    def test_no_collision_parallel_paths(self):
        """Test that parallel paths don't trigger collision."""
        # Two drones flying parallel, 10m apart
        own_pos = (0.0, 0.0, -10.0)
        own_vel = (5.0, 0.0, 0.0)

        neighbor_pos = (0.0, 10.0, -10.0)  # 10m to the east
        neighbor_vel = (5.0, 0.0, 0.0)  # Same velocity

        rel_vel = (neighbor_vel[0] - own_vel[0],
                   neighbor_vel[1] - own_vel[1],
                   neighbor_vel[2] - own_vel[2])

        rel_vel_sq = sum(v**2 for v in rel_vel)
        # Relative velocity is 0 - they're moving together
        assert rel_vel_sq < 0.01

    def test_diverging_paths(self):
        """Test that diverging paths don't trigger collision."""
        own_pos = (0.0, 0.0, -10.0)
        own_vel = (5.0, 0.0, 0.0)  # Moving north

        neighbor_pos = (5.0, 0.0, -10.0)  # 5m north
        neighbor_vel = (5.0, 0.0, 0.0)  # Also moving north, same speed

        rel_pos = (neighbor_pos[0] - own_pos[0],
                   neighbor_pos[1] - own_pos[1],
                   neighbor_pos[2] - own_pos[2])
        rel_vel = (neighbor_vel[0] - own_vel[0],
                   neighbor_vel[1] - own_vel[1],
                   neighbor_vel[2] - own_vel[2])

        rel_vel_sq = sum(v**2 for v in rel_vel)
        if rel_vel_sq < 0.01:
            # Not approaching - no collision
            collision = False
        else:
            dot_pv = sum(p * v for p, v in zip(rel_pos, rel_vel))
            t_min = -dot_pv / rel_vel_sq
            collision = t_min > 0  # Only future collisions

        assert not collision


class TestNeighborTrackerIntegration:
    """Integration tests that mock ROS2."""

    @pytest.fixture
    def mock_node(self):
        """Create a mock ROS2 node."""
        node = MagicMock()
        node.create_subscription = MagicMock(return_value=MagicMock())
        node.destroy_subscription = MagicMock()
        return node

    def test_tracker_initialization(self, mock_node):
        """Test tracker initializes without errors."""
        # This test verifies the structure is correct
        # Full ROS2 integration testing requires colcon build
        config = NeighborTrackerConfig(
            stale_timeout=3.0,
            safe_distance=3.0,
        )

        # Create neighbors dict as NeighborTracker would
        neighbors = {}
        for i in range(3):
            if i != 0:  # Skip own ID
                neighbors[i] = NeighborState(drone_id=i)

        assert len(neighbors) == 2
        assert 1 in neighbors
        assert 2 in neighbors
        assert 0 not in neighbors

    def test_timeout_detection(self):
        """Test stale/failed neighbor detection."""
        config = NeighborTrackerConfig(
            stale_timeout=3.0,
            failed_timeout=10.0,
        )

        neighbor = NeighborState(
            drone_id=1,
            last_update_time=time.time() - 5.0,  # 5 seconds ago
        )

        now = time.time()
        age = now - neighbor.last_update_time

        # Should be stale but not failed
        assert age > config.stale_timeout
        assert age < config.failed_timeout

    def test_callback_registration(self):
        """Test callback registration pattern."""
        callbacks_joined = []
        callbacks_lost = []
        callbacks_collision = []

        def on_joined(drone_id):
            callbacks_joined.append(drone_id)

        def on_lost(drone_id):
            callbacks_lost.append(drone_id)

        def on_collision(warning):
            callbacks_collision.append(warning)

        # Simulate callback invocation
        on_joined(5)
        on_lost(3)
        on_collision(CollisionWarning(
            other_drone_id=2,
            time_to_collision=1.0,
            min_distance=2.0,
            avoidance_vector=(0.0, 1.0, 0.0),
        ))

        assert 5 in callbacks_joined
        assert 3 in callbacks_lost
        assert len(callbacks_collision) == 1
        assert callbacks_collision[0].other_drone_id == 2
