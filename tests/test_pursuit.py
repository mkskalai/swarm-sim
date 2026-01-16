"""Unit tests for pursuit controller."""

import math
import time
import pytest
from swarm.coordination.pursuit_controller import (
    PursuitStrategy,
    PursuerRole,
    PursuitTarget,
    PursuitConfig,
    PursuerState,
    PursuitController,
    SimulatedTarget,
)


class TestPursuitTarget:
    """Tests for PursuitTarget."""

    def test_speed_calculation(self):
        """Speed should be calculated from velocity."""
        target = PursuitTarget(
            target_id=1,
            position=(0, 0, 0),
            velocity=(3.0, 4.0, 0.0),
        )
        assert target.speed == 5.0

    def test_horizontal_speed(self):
        """Horizontal speed should ignore vertical component."""
        target = PursuitTarget(
            target_id=1,
            position=(0, 0, 0),
            velocity=(3.0, 4.0, 5.0),
        )
        assert target.horizontal_speed == 5.0

    def test_predict_position(self):
        """Position prediction with constant velocity."""
        target = PursuitTarget(
            target_id=1,
            position=(0.0, 0.0, 0.0),
            velocity=(10.0, 5.0, 0.0),
        )

        predicted = target.predict_position(2.0)
        assert predicted[0] == 20.0  # 10 * 2
        assert predicted[1] == 10.0  # 5 * 2

    def test_predict_position_no_velocity(self):
        """Prediction without velocity should return current position."""
        target = PursuitTarget(
            target_id=1,
            position=(10.0, 20.0, -5.0),
            velocity=None,
        )

        predicted = target.predict_position(5.0)
        assert predicted == target.position


class TestPursuitController:
    """Tests for PursuitController."""

    def test_initialization(self):
        """Controller should initialize correctly."""
        config = PursuitConfig(
            strategy=PursuitStrategy.SURROUND,
            surround_radius=20.0,
        )

        controller = PursuitController(num_drones=4, config=config)
        assert not controller.is_tracking

    def test_set_target(self):
        """Setting target should activate tracking."""
        controller = PursuitController(
            num_drones=4,
            config=PursuitConfig(),
        )

        target = PursuitTarget(
            target_id=1,
            position=(50.0, 50.0, -1.5),
            class_name="person",
        )

        controller.set_target(target)
        assert controller.is_tracking
        assert controller.target_position == (50.0, 50.0, -1.5)

    def test_assign_pursuers_surround(self):
        """Surround strategy should assign correct roles."""
        config = PursuitConfig(strategy=PursuitStrategy.SURROUND)
        controller = PursuitController(num_drones=4, config=config)

        assignments = controller.assign_pursuers([0, 1, 2, 3])

        assert len(assignments) == 4
        assert PursuerRole.LEAD in assignments.values()
        assert PursuerRole.FLANK_LEFT in assignments.values()
        assert PursuerRole.FLANK_RIGHT in assignments.values()
        assert PursuerRole.REAR in assignments.values()

    def test_assign_pursuers_follow(self):
        """Follow strategy should assign lead and rear."""
        config = PursuitConfig(strategy=PursuitStrategy.FOLLOW)
        controller = PursuitController(num_drones=3, config=config)

        assignments = controller.assign_pursuers([0, 1, 2])

        assert assignments[0] == PursuerRole.LEAD
        assert assignments[1] == PursuerRole.REAR
        assert assignments[2] == PursuerRole.REAR

    def test_get_pursuit_targets_surround(self):
        """Surround should position drones around target."""
        config = PursuitConfig(
            strategy=PursuitStrategy.SURROUND,
            surround_radius=20.0,
            min_altitude=10.0,
        )
        controller = PursuitController(num_drones=4, config=config)

        target = PursuitTarget(
            target_id=1,
            position=(50.0, 50.0, -1.5),
        )
        controller.set_target(target)
        controller.assign_pursuers([0, 1, 2, 3])

        drone_positions = {i: (0.0, 0.0, -15.0) for i in range(4)}
        pursuit_targets = controller.get_pursuit_targets(drone_positions)

        assert len(pursuit_targets) == 4

        # All targets should be at surround radius from target
        for drone_id, pos in pursuit_targets.items():
            dist = math.sqrt(
                (pos[0] - target.position[0])**2 +
                (pos[1] - target.position[1])**2
            )
            assert abs(dist - config.surround_radius) < 1.0

    def test_update_target_position(self):
        """Updating target should update internal state."""
        controller = PursuitController(
            num_drones=2,
            config=PursuitConfig(),
        )

        target = PursuitTarget(
            target_id=1,
            position=(0.0, 0.0, -1.5),
        )
        controller.set_target(target)

        controller.update_target_position((10.0, 5.0, -1.5))

        assert controller.target.position == (10.0, 5.0, -1.5)

    def test_velocity_estimation(self):
        """Velocity should be estimated from position history."""
        controller = PursuitController(
            num_drones=2,
            config=PursuitConfig(),
        )

        target = PursuitTarget(
            target_id=1,
            position=(0.0, 0.0, -1.5),
        )
        controller.set_target(target)

        # Update positions over time
        controller.update_target_position((0.0, 0.0, -1.5), timestamp=0.0)
        controller.update_target_position((10.0, 0.0, -1.5), timestamp=1.0)

        # Velocity should be estimated
        assert controller.target.velocity is not None
        assert controller.target.velocity[0] == pytest.approx(10.0, abs=1.0)

    def test_handle_target_lost(self):
        """Lost target should return search pattern."""
        controller = PursuitController(
            num_drones=2,
            config=PursuitConfig(target_lost_timeout=1.0),
        )

        target = PursuitTarget(
            target_id=1,
            position=(50.0, 50.0, -1.5),
            velocity=(2.0, 1.0, 0.0),
        )
        controller.set_target(target)

        search_config = controller.handle_target_lost()

        assert search_config is not None
        assert controller.target is None

    def test_clear_target(self):
        """Clearing target should stop tracking."""
        controller = PursuitController(
            num_drones=2,
            config=PursuitConfig(),
        )

        target = PursuitTarget(target_id=1, position=(0, 0, 0))
        controller.set_target(target)
        assert controller.is_tracking

        controller.clear_target()
        assert not controller.is_tracking


class TestSimulatedTarget:
    """Tests for SimulatedTarget."""

    def test_initialization(self):
        """Target should initialize at start position."""
        target = SimulatedTarget(
            start_position=(0.0, 0.0, -1.5),
            waypoints=[(50.0, 0.0, -1.5)],
            speed=5.0,
        )

        assert target.position == (0.0, 0.0, -1.5)
        assert not target.is_complete

    def test_movement(self):
        """Target should move toward waypoints."""
        target = SimulatedTarget(
            start_position=(0.0, 0.0, -1.5),
            waypoints=[(50.0, 0.0, -1.5)],
            speed=10.0,
        )

        target.update(1.0)  # 1 second at 10 m/s

        assert target.position[0] == pytest.approx(10.0, abs=0.5)
        assert target.position[1] == pytest.approx(0.0, abs=0.1)

    def test_reaches_waypoints(self):
        """Target should reach waypoints."""
        target = SimulatedTarget(
            start_position=(0.0, 0.0, -1.5),
            waypoints=[(10.0, 0.0, -1.5)],
            speed=10.0,
        )

        # Move to waypoint
        for _ in range(20):
            target.update(0.1)

        assert target.is_complete

    def test_loop_mode(self):
        """Loop mode should restart from first waypoint."""
        target = SimulatedTarget(
            start_position=(0.0, 0.0, 0.0),
            waypoints=[(5.0, 0.0, 0.0)],
            speed=10.0,
            loop=True,
        )

        # Move past waypoint
        for _ in range(20):
            target.update(0.1)

        assert not target.is_complete

    def test_velocity_direction(self):
        """Velocity should point toward current waypoint."""
        target = SimulatedTarget(
            start_position=(0.0, 0.0, -1.5),
            waypoints=[(50.0, 0.0, -1.5)],
            speed=5.0,
        )

        target.update(0.1)

        # Moving east (positive x direction)
        assert target.velocity[0] > 0

    def test_heading(self):
        """Heading should match velocity direction."""
        target = SimulatedTarget(
            start_position=(0.0, 0.0, -1.5),
            waypoints=[(0.0, 50.0, -1.5)],  # Moving north
            speed=5.0,
        )

        target.update(0.1)

        # Heading toward north (east component positive)
        assert 0 <= target.heading <= 90 or target.heading > 350

    def test_to_pursuit_target(self):
        """Should convert to PursuitTarget."""
        sim = SimulatedTarget(
            start_position=(10.0, 20.0, -1.5),
            waypoints=[(50.0, 20.0, -1.5)],
            speed=3.0,
        )

        sim.update(0.1)
        pursuit = sim.to_pursuit_target(target_id=42)

        assert pursuit.target_id == 42
        assert pursuit.position == sim.position
        assert pursuit.velocity == sim.velocity
        assert pursuit.class_name == "simulated"
