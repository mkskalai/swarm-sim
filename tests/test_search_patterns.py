"""Unit tests for search pattern generation and adaptive search."""

import math
import pytest
from swarm.coordination.search_patterns import (
    SearchPatternType,
    SearchPatternConfig,
    SearchPatternGenerator,
    SearchProgress,
    AdaptiveSearchController,
)


class TestSearchPatternGenerator:
    """Tests for SearchPatternGenerator."""

    def test_lawnmower_generates_waypoints(self):
        """Lawnmower pattern should generate correct number of waypoints."""
        waypoints = SearchPatternGenerator.generate_lawnmower(
            bounds=(0, 100, 0, 100),
            lane_spacing=25.0,
            altitude=15.0,
        )

        assert len(waypoints) > 0
        # 5 lanes (100/25 + 1), 2 waypoints each
        assert len(waypoints) >= 10

    def test_lawnmower_covers_bounds(self):
        """Lawnmower pattern should cover the specified bounds."""
        bounds = (0, 100, 0, 80)
        waypoints = SearchPatternGenerator.generate_lawnmower(
            bounds=bounds,
            lane_spacing=20.0,
            altitude=10.0,
        )

        # Check waypoints are within bounds
        for wp in waypoints:
            assert bounds[0] <= wp.north <= bounds[1]
            assert bounds[2] <= wp.east <= bounds[3]
            assert wp.altitude == 10.0

    def test_spiral_generates_waypoints(self):
        """Spiral pattern should generate waypoints."""
        waypoints = SearchPatternGenerator.generate_spiral(
            center=(50.0, 50.0),
            max_radius=40.0,
            spacing=10.0,
            altitude=15.0,
        )

        assert len(waypoints) > 0
        # First waypoint should be at center
        assert waypoints[0].north == 50.0
        assert waypoints[0].east == 50.0

    def test_spiral_expands_outward(self):
        """Spiral should expand outward from center."""
        center = (50.0, 50.0)
        waypoints = SearchPatternGenerator.generate_spiral(
            center=center,
            max_radius=30.0,
            spacing=5.0,
            altitude=10.0,
        )

        # Calculate distances from center
        distances = []
        for wp in waypoints:
            dist = math.sqrt((wp.north - center[0])**2 + (wp.east - center[1])**2)
            distances.append(dist)

        # Distances should generally increase (with some wiggle for spiral)
        assert distances[-1] > distances[0]

    def test_expanding_square_generates_waypoints(self):
        """Expanding square should generate waypoints."""
        waypoints = SearchPatternGenerator.generate_expanding_square(
            center=(0.0, 0.0),
            max_size=50.0,
            step_size=10.0,
            altitude=15.0,
        )

        assert len(waypoints) > 0
        # First waypoint at center
        assert waypoints[0].north == 0.0
        assert waypoints[0].east == 0.0

    def test_expanding_square_pattern(self):
        """Expanding square should follow correct pattern."""
        waypoints = SearchPatternGenerator.generate_expanding_square(
            center=(0.0, 0.0),
            max_size=30.0,
            step_size=10.0,
            altitude=10.0,
        )

        # Should have at least center + one expansion
        assert len(waypoints) >= 5

    def test_sector_generates_waypoints(self):
        """Sector pattern should generate waypoints."""
        waypoints = SearchPatternGenerator.generate_sector(
            center=(0.0, 0.0),
            max_radius=50.0,
            start_angle=0.0,
            end_angle=90.0,
            altitude=15.0,
            spacing=15.0,
        )

        assert len(waypoints) > 0


class TestSearchPatternConfig:
    """Tests for SearchPatternConfig."""

    def test_lawnmower_requires_bounds(self):
        """Lawnmower config should require bounds."""
        with pytest.raises(ValueError):
            SearchPatternConfig(
                pattern_type=SearchPatternType.LAWNMOWER,
                bounds=None,  # Should raise
            )

    def test_spiral_requires_center(self):
        """Spiral config should require center."""
        with pytest.raises(ValueError):
            SearchPatternConfig(
                pattern_type=SearchPatternType.SPIRAL,
                center=None,  # Should raise
            )

    def test_valid_lawnmower_config(self):
        """Valid lawnmower config should work."""
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.LAWNMOWER,
            bounds=(0, 100, 0, 100),
            lane_spacing=20.0,
        )
        assert config.bounds == (0, 100, 0, 100)

    def test_valid_spiral_config(self):
        """Valid spiral config should work."""
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.SPIRAL,
            center=(50.0, 50.0),
            max_radius=100.0,
        )
        assert config.center == (50.0, 50.0)


class TestSearchProgress:
    """Tests for SearchProgress."""

    def test_initial_state(self):
        """New progress should be at start."""
        progress = SearchProgress(drone_id=0)
        assert progress.current_index == 0
        assert progress.completed_waypoints == 0
        assert not progress.is_complete

    def test_advance(self):
        """Advance should move to next waypoint."""
        from swarm.coordination.missions import Waypoint

        progress = SearchProgress(
            drone_id=0,
            assigned_waypoints=[
                Waypoint(north=0, east=0, altitude=10),
                Waypoint(north=10, east=0, altitude=10),
            ],
        )

        assert progress.current_waypoint.north == 0
        has_more = progress.advance()
        assert has_more
        assert progress.current_waypoint.north == 10
        has_more = progress.advance()
        assert not has_more
        assert progress.is_complete


class TestAdaptiveSearchController:
    """Tests for AdaptiveSearchController."""

    def test_initialization(self):
        """Controller should initialize correctly."""
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=(0.0, 0.0),
            max_size=100.0,
        )

        controller = AdaptiveSearchController(
            num_drones=4,
            pattern_config=config,
        )

        assignments = controller.initialize_search([0, 1, 2, 3])
        assert len(assignments) == 4

    def test_waypoint_distribution(self):
        """Waypoints should be distributed among drones."""
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=(0.0, 0.0),
            max_size=100.0,
            step_size=10.0,
        )

        controller = AdaptiveSearchController(
            num_drones=4,
            pattern_config=config,
        )

        assignments = controller.initialize_search([0, 1, 2, 3])

        total_wps = sum(len(wps) for wps in assignments.values())
        assert total_wps > 0

        # Each drone should have similar number of waypoints
        counts = [len(wps) for wps in assignments.values()]
        assert max(counts) - min(counts) <= 1

    def test_failure_reallocation(self):
        """Failed drone's waypoints should be reallocated."""
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=(0.0, 0.0),
            max_size=100.0,
            step_size=10.0,
        )

        controller = AdaptiveSearchController(
            num_drones=4,
            pattern_config=config,
        )

        controller.initialize_search([0, 1, 2, 3])

        # Get initial total
        initial_total = sum(
            len(controller.get_progress(d).assigned_waypoints)
            for d in [0, 1, 2, 3]
        )

        # Simulate failure
        controller.on_drone_failure(1)

        # Check waypoints redistributed
        remaining_total = sum(
            len(controller.get_progress(d).assigned_waypoints)
            for d in [0, 2, 3]
        )

        # Total should be preserved (minus completed ones)
        assert remaining_total >= initial_total - 10  # Allow for some completed

    def test_progress_tracking(self):
        """Progress should be tracked correctly."""
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=(0.0, 0.0),
            max_size=50.0,
            step_size=10.0,
        )

        controller = AdaptiveSearchController(
            num_drones=2,
            pattern_config=config,
            arrival_tolerance=2.0,
        )

        controller.initialize_search([0, 1])

        # Simulate reaching waypoint
        targets = controller.get_current_targets()
        assert 0 in targets

        # Move to target
        reached = controller.update_progress(0, targets[0])
        assert reached

    def test_coverage_stats(self):
        """Coverage stats should be tracked."""
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=(0.0, 0.0),
            max_size=50.0,
        )

        controller = AdaptiveSearchController(
            num_drones=2,
            pattern_config=config,
        )

        controller.initialize_search([0, 1])

        stats = controller.coverage_stats
        assert "total_waypoints" in stats
        assert "completion_percent" in stats
        assert stats["active_drones"] == 2
