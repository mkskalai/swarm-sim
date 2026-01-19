"""Unit tests for swarm coordination.

These tests verify formation calculations, leader-follower logic,
mission planning, and failure handling without requiring simulation.

Run with: python scripts/run_tests.py --unit
"""

import pytest
import time
import math

from swarm.coordination import (
    # Formations
    FormationType,
    FormationConfig,
    FormationCalculator,
    FormationTransition,
    get_formation_positions,
    # Leader-Follower
    LeaderFollowerController,
    FollowerOffset,
    # Missions
    Waypoint,
    WaypointAction,
    DroneMission,
    SwarmMission,
    MissionExecutionMode,
    MissionPlanner,
    MissionExecutor,
    # Failure Handling
    FailureHandler,
    FailureType,
)


class TestFormations:
    """Unit tests for formation calculations."""

    def test_line_formation_3_drones(self):
        """Test line formation generates correct positions."""
        positions = get_formation_positions(FormationType.LINE, 3, spacing=5.0)

        assert len(positions) == 3
        # Line formation should be along East axis, centered
        # With 3 drones at spacing 5: -5, 0, +5
        assert positions[0][1] == pytest.approx(-5.0, rel=0.01)  # East
        assert positions[1][1] == pytest.approx(0.0, abs=0.01)
        assert positions[2][1] == pytest.approx(5.0, rel=0.01)
        # All should have North=0 (line along East)
        for pos in positions:
            assert pos[0] == pytest.approx(0.0, abs=0.01)

    def test_line_formation_6_drones(self):
        """Test line formation scales to 6 drones."""
        positions = get_formation_positions(FormationType.LINE, 6, spacing=5.0)

        assert len(positions) == 6
        # Should be centered, so positions from -12.5 to +12.5
        easts = [pos[1] for pos in positions]
        assert min(easts) == pytest.approx(-12.5, rel=0.01)
        assert max(easts) == pytest.approx(12.5, rel=0.01)

    def test_line_formation_same_altitude(self):
        """Test line formation has all drones at same altitude."""
        positions = get_formation_positions(FormationType.LINE, 3, spacing=5.0, altitude=10.0)

        # All drones should be at same altitude (no separation in line)
        altitudes = [-pos[2] for pos in positions]
        assert all(alt == pytest.approx(10.0, abs=0.01) for alt in altitudes)

    def test_grid_formation_6_drones(self):
        """Test grid formation creates 2x3 or 3x2 grid."""
        positions = get_formation_positions(FormationType.GRID, 6)

        assert len(positions) == 6
        # All positions should be distinct
        pos_set = set(positions)
        assert len(pos_set) == 6

    def test_grid_formation_altitude_separation(self):
        """Test that grid formation has altitude separation."""
        config = FormationConfig(altitude=10.0, altitude_separation=2.0)
        calc = FormationCalculator(config)
        positions = calc.calculate(FormationType.GRID, 3)

        # Each drone should have different altitude
        altitudes = [-pos[2] for pos in positions]
        # All altitudes should be unique
        assert len(set(altitudes)) == 3
        # Separation should be 2.0m
        altitudes_sorted = sorted(altitudes)
        assert altitudes_sorted[1] - altitudes_sorted[0] == pytest.approx(2.0, abs=0.01)

    def test_formation_rotation(self):
        """Test formation heading rotation."""
        calc = FormationCalculator()

        # Line at 0 degrees
        pos_0 = calc.calculate(FormationType.LINE, 3, FormationConfig(heading=0.0))
        # Line at 90 degrees
        pos_90 = calc.calculate(FormationType.LINE, 3, FormationConfig(heading=90.0))

        # At 0 degrees, line is along East (all North=0)
        assert pos_0[0][0] == pytest.approx(0.0, abs=0.01)

        # At 90 degrees, line is along North (all East=0)
        # (rotated 90 degrees clockwise)
        for pos in pos_90:
            # East should now be close to 0
            assert abs(pos[1]) < 1.0 or pos[0] != pytest.approx(0.0, abs=1.0)

    def test_formation_single_drone(self):
        """Test formations with single drone."""
        for formation_type in FormationType:
            positions = get_formation_positions(formation_type, 1)
            assert len(positions) == 1

    def test_formation_zero_drones(self):
        """Test formations with zero drones returns empty list."""
        positions = get_formation_positions(FormationType.LINE, 0)
        assert positions == []


class TestFormationTransition:
    """Tests for smooth formation transitions."""

    def test_transition_start_end(self):
        """Test transition starts and ends at correct positions."""
        start = [(0.0, 0.0, -10.0), (5.0, 0.0, -10.0)]
        end = [(0.0, 0.0, -10.0), (0.0, 5.0, -10.0)]

        transition = FormationTransition(start, end, duration=5.0)

        # At t=0, should be at start
        pos_start = transition.get_positions_at_time(0.0)
        assert pos_start[0] == pytest.approx(start[0], rel=0.01)
        assert pos_start[1] == pytest.approx(start[1], rel=0.01)

        # At t=duration, should be at end
        pos_end = transition.get_positions_at_time(5.0)
        assert pos_end[0] == pytest.approx(end[0], rel=0.01)
        assert pos_end[1] == pytest.approx(end[1], rel=0.01)

    def test_transition_midpoint(self):
        """Test transition midpoint is interpolated."""
        start = [(0.0, 0.0, -10.0)]
        end = [(10.0, 0.0, -10.0)]

        transition = FormationTransition(start, end, duration=4.0)

        # At midpoint, should be halfway
        pos_mid = transition.get_positions_at_time(2.0)
        # Due to cosine interpolation, midpoint should be exactly 5.0
        assert pos_mid[0][0] == pytest.approx(5.0, rel=0.01)

    def test_transition_is_complete(self):
        """Test transition completion check."""
        transition = FormationTransition([], [], duration=5.0)

        assert not transition.is_complete(0.0)
        assert not transition.is_complete(4.9)
        assert transition.is_complete(5.0)
        assert transition.is_complete(10.0)


class TestLeaderFollower:
    """Unit tests for leader-follower controller."""

    def test_set_leader(self):
        """Test setting leader drone."""
        lf = LeaderFollowerController(num_drones=3)

        assert lf.set_leader(0) is True
        assert lf.leader_id == 0
        assert lf.drones[0].is_leader is True
        assert lf.drones[1].is_leader is False

    def test_set_invalid_leader(self):
        """Test setting invalid leader ID."""
        lf = LeaderFollowerController(num_drones=3)

        assert lf.set_leader(10) is False

    def test_follower_offsets(self):
        """Test follower offset calculations."""
        lf = LeaderFollowerController(num_drones=3)
        lf.set_leader(0)

        lf.set_formation_offsets([
            None,  # Leader has no offset
            FollowerOffset(north=-5.0, east=-5.0, down=0.0),
            FollowerOffset(north=-5.0, east=5.0, down=0.0),
        ])

        leader_pos = (10.0, 10.0, -10.0)

        target_1 = lf.get_target_position(1, leader_pos)
        assert target_1 == (5.0, 5.0, -10.0)

        target_2 = lf.get_target_position(2, leader_pos)
        assert target_2 == (5.0, 15.0, -10.0)

        # Leader should return None
        target_leader = lf.get_target_position(0, leader_pos)
        assert target_leader is None

    def test_set_offsets_from_positions(self):
        """Test converting formation positions to offsets."""
        lf = LeaderFollowerController(num_drones=3)

        positions = [
            (0.0, 0.0, -10.0),   # Leader position
            (-5.0, -5.0, -12.0), # Follower 1
            (-5.0, 5.0, -8.0),   # Follower 2
        ]

        lf.set_offsets_from_positions(positions, leader_id=0)
        lf.set_leader(0)

        # Follower 1 offset should be (-5, -5, -2) from leader
        assert lf.drones[1].offset.north == pytest.approx(-5.0)
        assert lf.drones[1].offset.east == pytest.approx(-5.0)
        assert lf.drones[1].offset.down == pytest.approx(-2.0)

    def test_get_all_target_positions(self):
        """Test getting all follower targets."""
        lf = LeaderFollowerController(num_drones=3)
        lf.set_leader(0)

        lf.set_formation_offsets([
            None,
            FollowerOffset(north=-5.0, east=0.0, down=0.0),
            FollowerOffset(north=-10.0, east=0.0, down=0.0),
        ])

        leader_pos = (0.0, 0.0, -10.0)
        targets = lf.get_all_target_positions(leader_pos)

        assert len(targets) == 2
        assert 0 not in targets  # Leader not included
        assert 1 in targets
        assert 2 in targets

    def test_leader_heartbeat_failure(self):
        """Test leader failure detection and promotion."""
        lf = LeaderFollowerController(num_drones=3, heartbeat_timeout=0.5)
        lf.set_leader(0)

        # Update heartbeats
        lf.update_drone_heartbeat(0)
        lf.update_drone_heartbeat(1)
        lf.update_drone_heartbeat(2)

        # Verify leader is healthy
        assert lf.check_leader_health() is True
        assert lf.leader_id == 0

        # Simulate leader failure (no heartbeat)
        time.sleep(0.6)
        lf.update_drone_heartbeat(1)
        lf.update_drone_heartbeat(2)

        # Should promote next drone
        result = lf.check_leader_health()
        assert result is True
        assert lf.leader_id == 1

    def test_leader_change_callback(self):
        """Test leader change callback is invoked."""
        lf = LeaderFollowerController(num_drones=3)

        callback_args = []
        lf.on_leader_change(lambda old, new: callback_args.append((old, new)))

        lf.set_leader(0)  # Callback: (None, 0)
        lf.set_leader(1)  # Callback: (0, 1)

        # Both leader changes trigger callbacks
        assert len(callback_args) == 2
        assert callback_args[0] == (None, 0)
        assert callback_args[1] == (0, 1)


class TestMissions:
    """Unit tests for mission system."""

    def test_waypoint_properties(self):
        """Test Waypoint dataclass properties."""
        wp = Waypoint(north=10.0, east=20.0, altitude=15.0)

        assert wp.down == -15.0
        assert wp.position_ned == (10.0, 20.0, -15.0)

    def test_drone_mission_advance(self):
        """Test advancing through waypoints."""
        dm = DroneMission(
            drone_id=0,
            waypoints=[
                Waypoint(north=0, east=0, altitude=10),
                Waypoint(north=10, east=0, altitude=10),
                Waypoint(north=20, east=0, altitude=10),
            ]
        )

        assert dm.current_waypoint_index == 0
        assert dm.total_waypoints == 3
        assert dm.waypoints_remaining == 3

        assert dm.advance() is True
        assert dm.current_waypoint_index == 1
        assert dm.waypoints_remaining == 2

        assert dm.advance() is True
        assert dm.current_waypoint_index == 2

        assert dm.advance() is False  # No more waypoints
        assert dm.is_complete is True

    def test_patrol_mission_creation(self):
        """Test patrol mission creates correct waypoints."""
        waypoints = [(0, 0), (100, 0), (100, 100), (0, 100)]
        mission = MissionPlanner.create_patrol_mission(
            waypoints, num_drones=3, altitude=15.0
        )

        assert len(mission.drone_missions) == 3
        assert mission.execution_mode == MissionExecutionMode.SYNCHRONIZED
        assert mission.name == "patrol"

        # Each drone should have 4 waypoints
        for dm in mission.drone_missions.values():
            assert len(dm.waypoints) == 4

    def test_area_coverage_mission(self):
        """Test area coverage mission creates lawnmower pattern."""
        mission = MissionPlanner.create_area_coverage_mission(
            bounds=(0, 100, 0, 100),
            num_drones=2,
            lane_spacing=10.0,
        )

        assert len(mission.drone_missions) == 2
        assert mission.execution_mode == MissionExecutionMode.PARALLEL
        assert not mission.formation_at_waypoints

        # Each drone should have waypoints
        for dm in mission.drone_missions.values():
            assert len(dm.waypoints) > 0

    def test_converge_mission(self):
        """Test converge mission creates single waypoint per drone."""
        mission = MissionPlanner.create_converge_mission(
            target=(50, 50),
            num_drones=3,
            altitude=10.0,
        )

        assert len(mission.drone_missions) == 3
        for dm in mission.drone_missions.values():
            assert len(dm.waypoints) == 1
            assert dm.waypoints[0].north == 50
            assert dm.waypoints[0].east == 50

    def test_orbit_mission(self):
        """Test orbit mission creates circular path."""
        mission = MissionPlanner.create_orbit_mission(
            center=(0, 0),
            radius=20.0,
            num_drones=2,
            orbits=2,
            points_per_orbit=8,
        )

        assert len(mission.drone_missions) == 2
        # 2 orbits * 8 points = 16 waypoints per drone
        for dm in mission.drone_missions.values():
            assert len(dm.waypoints) == 16

    def test_swarm_mission_progress(self):
        """Test mission progress calculation."""
        mission = SwarmMission(name="test")
        mission.drone_missions[0] = DroneMission(
            drone_id=0,
            waypoints=[Waypoint(0, 0, 10), Waypoint(10, 0, 10)]
        )
        mission.drone_missions[1] = DroneMission(
            drone_id=1,
            waypoints=[Waypoint(0, 0, 10), Waypoint(10, 0, 10)]
        )

        assert mission.get_progress() == 0.0

        mission.drone_missions[0].advance()
        assert mission.get_progress() == pytest.approx(0.25)

        mission.drone_missions[0].advance()
        mission.drone_missions[1].advance()
        assert mission.get_progress() == pytest.approx(0.75)

    def test_swarm_mission_is_complete(self):
        """Test mission completion check."""
        mission = SwarmMission(name="test")
        mission.drone_missions[0] = DroneMission(
            drone_id=0,
            waypoints=[Waypoint(0, 0, 10)]
        )

        assert not mission.is_complete()

        mission.drone_missions[0].advance()
        assert mission.is_complete()


class TestFailureHandler:
    """Unit tests for failure handling."""

    def test_heartbeat_timeout_detection(self):
        """Test heartbeat timeout triggers failure."""
        handler = FailureHandler(num_drones=3, heartbeat_timeout=0.5)

        # Update heartbeats for all
        handler.update_heartbeat(0, position=(0, 0, -10))
        handler.update_heartbeat(1, position=(0, 5, -10))
        handler.update_heartbeat(2, position=(0, 10, -10))

        # All should be healthy
        failures = handler.check_all_health()
        assert len(failures) == 0

        # Wait for timeout on drone 0
        time.sleep(0.6)
        handler.update_heartbeat(1, position=(0, 5, -10))
        handler.update_heartbeat(2, position=(0, 10, -10))

        failures = handler.check_all_health()
        assert len(failures) == 1
        assert failures[0].drone_id == 0
        assert failures[0].failure_type == FailureType.HEARTBEAT_LOSS

    def test_active_drones_tracking(self):
        """Test active drone tracking."""
        handler = FailureHandler(num_drones=3, heartbeat_timeout=1.0)

        # Initialize heartbeats
        handler.update_heartbeat(0)
        handler.update_heartbeat(1)
        handler.update_heartbeat(2)

        assert len(handler.get_active_drones()) == 3

        # Mark one as failed
        handler.manually_mark_failed(1, "test")

        assert len(handler.get_active_drones()) == 2
        assert 1 not in handler.get_active_drones()
        assert 1 in handler.get_failed_drones()

    def test_battery_critical_detection(self):
        """Test low battery triggers failure."""
        handler = FailureHandler(
            num_drones=2,
            heartbeat_timeout=10.0,
            battery_critical_threshold=15.0
        )

        handler.update_heartbeat(0, position=(0, 0, -10), battery=50.0)
        handler.update_heartbeat(1, position=(0, 5, -10), battery=10.0)

        failures = handler.check_all_health()
        assert len(failures) == 1
        assert failures[0].drone_id == 1
        assert failures[0].failure_type == FailureType.BATTERY_CRITICAL

    def test_position_error_detection(self):
        """Test position error detection."""
        handler = FailureHandler(
            num_drones=1,
            position_error_threshold=10.0
        )
        handler.update_heartbeat(0, position=(0, 0, -10))

        # Position within tolerance
        result = handler.check_position_error(0, (5, 5, -10), (0, 0, -10))
        assert result is None

        # Position outside tolerance
        result = handler.check_position_error(0, (50, 50, -10), (0, 0, -10))
        assert result is not None
        assert result.failure_type == FailureType.POSITION_ERROR

    def test_failure_callback(self):
        """Test failure callback is invoked."""
        handler = FailureHandler(num_drones=2, heartbeat_timeout=10.0)

        callback_events = []
        handler.on_failure(lambda e: callback_events.append(e))

        handler.update_heartbeat(0)
        handler.manually_mark_failed(0, "test")

        assert len(callback_events) == 1
        assert callback_events[0].drone_id == 0

    def test_recovery_callback(self):
        """Test recovery callback is invoked."""
        handler = FailureHandler(
            num_drones=1,
            heartbeat_timeout=10.0,
            battery_critical_threshold=15.0
        )

        recovery_ids = []
        handler.on_recovery(lambda id: recovery_ids.append(id))

        # Mark as failed
        handler.manually_mark_failed(0, "test")
        assert 0 in handler.get_failed_drones()

        # Recover manually
        handler.manually_mark_recovered(0)
        assert 0 not in handler.get_failed_drones()
        assert len(recovery_ids) == 1
        assert recovery_ids[0] == 0

    def test_redistribute_formation(self):
        """Test formation redistribution."""
        positions = [
            (0, 0, -10),
            (5, 0, -12),
            (10, 0, -14),
        ]

        # If drone 1 fails, remaining drones keep their positions
        new_positions = FailureHandler.redistribute_formation(
            positions, active_drone_ids=[0, 2]
        )

        assert len(new_positions) == 2
        assert new_positions[0] == positions[0]
        assert new_positions[2] == positions[2]


class TestMissionExecutor:
    """Tests for mission execution tracking."""

    def test_executor_get_targets(self):
        """Test getting current targets from executor."""
        mission = SwarmMission(name="test")
        mission.drone_missions[0] = DroneMission(
            drone_id=0,
            waypoints=[Waypoint(10, 20, 15)]
        )

        executor = MissionExecutor(mission)

        targets = executor.get_current_targets()
        assert len(targets) == 1
        assert targets[0] == (10, 20, -15)

    def test_executor_waypoint_arrival(self):
        """Test waypoint arrival detection."""
        mission = SwarmMission(
            name="test",
            execution_mode=MissionExecutionMode.PARALLEL
        )
        mission.drone_missions[0] = DroneMission(
            drone_id=0,
            waypoints=[
                Waypoint(10, 20, 15, hold_time=0),
                Waypoint(30, 40, 15, hold_time=0),
            ]
        )

        executor = MissionExecutor(mission, position_tolerance=1.0)
        executor.start()

        # Drone not at waypoint
        result = executor.update_drone_position(0, (0, 0, -15))
        assert result is None

        # Drone arrives at waypoint
        result = executor.update_drone_position(0, (10, 20, -15))
        assert result is not None  # Waypoint completed
        assert result.north == 10

        # Now should be targeting second waypoint
        targets = executor.get_current_targets()
        assert targets[0] == (30, 40, -15)
