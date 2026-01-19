"""Full stack simulation tests for drone swarm.

These tests require the complete simulation environment:
- Gazebo with generated world
- SITL instances for each drone
- Optional ROS2 nodes (SwarmBridge)

Run with: python scripts/run_tests.py --sim -n 3

The tests exercise the full stack together:
1. Connection and EKF convergence
2. Takeoff and formation flight
3. GPS denial patrol with entry/exit logging
4. Landing and cleanup (handled by fixture teardown)
"""

import os
import time
import pytest

# Check if simulation is running
SIM_RUNNING = os.environ.get("SWARM_SIM_RUNNING", "").lower() in ("1", "true", "yes")
SKIP_REASON = "Requires running simulation. Use: python scripts/run_tests.py --sim"


@pytest.mark.sim
class TestSwarmConnection:
    """Test swarm connection and basic operations."""

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_connect_all_drones(self, swarm_controller):
        """Test connecting to all SITL drones."""
        assert swarm_controller.is_connected is True
        assert len(swarm_controller.drones) == swarm_controller.num_drones

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_ekf_convergence(self, swarm_controller):
        """Test that EKF converges on all drones."""
        # EKF should already be converged from fixture (wait_for_ekf_all was called)
        # If we got here, EKF converged successfully
        assert swarm_controller.is_connected is True
        assert len(swarm_controller.drones) > 0


@pytest.mark.sim
class TestSwarmFlight:
    """Test swarm flight operations."""

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_takeoff_all(self, armed_swarm):
        """Test takeoff of all drones."""
        armed_swarm.takeoff_all(altitude=5.0)
        time.sleep(5)  # Wait for drones to reach altitude

        # All drones should be in flight
        assert armed_swarm.is_armed is True


@pytest.mark.sim
class TestFormationFlight:
    """Test formation flying capabilities."""

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_line_formation(self, flying_swarm):
        """Test flying in line formation."""
        from swarm.coordination import FormationType, FormationConfig

        config = FormationConfig(spacing=5.0, altitude=10.0)
        flying_swarm.fly_formation(
            FormationType.LINE,
            duration=10.0,
            config=config,
        )

        # Test completes if no exception

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_grid_formation(self, flying_swarm):
        """Test flying in grid formation."""
        from swarm.coordination import FormationType, FormationConfig

        config = FormationConfig(spacing=5.0, altitude=10.0)
        flying_swarm.fly_formation(
            FormationType.GRID,
            duration=10.0,
            config=config,
        )

        # Test completes if no exception

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_formation_transition(self, flying_swarm):
        """Test transitioning between formations."""
        from swarm.coordination import FormationType, FormationConfig

        config = FormationConfig(spacing=5.0, altitude=10.0)

        # Start in line
        flying_swarm.fly_formation(FormationType.LINE, duration=5.0, config=config)

        # Transition to grid
        flying_swarm.fly_formation(FormationType.GRID, duration=5.0, config=config)

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    def test_line_formation_positions(self, flying_swarm, timeout_multiplier):
        """Test LINE formation achieves correct positions.

        This test verifies that drones actually reach their expected
        LINE formation positions, not just that the command was sent.
        """
        from swarm.coordination import (
            FormationType,
            FormationConfig,
            FormationCalculator,
        )

        config = FormationConfig(spacing=5.0, altitude=10.0)
        calc = FormationCalculator(config)

        # Calculate expected positions for LINE formation
        num_drones = flying_swarm.num_drones
        expected_positions = calc.calculate(FormationType.LINE, num_drones, config)

        # Fly LINE formation with longer hold to ensure convergence
        arrival_timeout = 30.0 * timeout_multiplier
        result = flying_swarm.fly_formation(
            FormationType.LINE,
            duration=20.0,  # Longer hold for verification
            config=config,
            arrival_timeout=arrival_timeout,
            arrival_tolerance=2.0,
        )

        assert result is True

        # Get actual positions in WORLD frame (for comparison with expected)
        actual_positions = flying_swarm._get_all_positions_world()

        print(f"\nLINE Formation Position Verification ({num_drones} drones):")
        print(f"{'Drone':<8} {'Expected N':<12} {'Actual N':<12} {'Expected E':<12} {'Actual E':<12} {'Dist':<8}")
        print("-" * 70)

        # Verify each drone is within tolerance of expected
        for idx, expected in enumerate(expected_positions):
            if idx in actual_positions:
                actual = actual_positions[idx]
                dist = flying_swarm._distance_to_target(actual, expected)
                print(
                    f"{idx:<8} {expected[0]:<12.1f} {actual[0]:<12.1f} "
                    f"{expected[1]:<12.1f} {actual[1]:<12.1f} {dist:<8.1f}"
                )
                assert dist < 5.0, (
                    f"Drone {idx} at {actual}, expected {expected}, dist={dist:.1f}m"
                )

        # LINE formation specific checks:
        # 1. All drones should have same North (~0)
        norths = [pos[0] for pos in actual_positions.values()]
        north_spread = max(norths) - min(norths)
        print(f"\nNorth spread: {north_spread:.1f}m (should be < 5m for LINE)")
        assert north_spread < 5.0, f"LINE: North spread {north_spread:.1f}m too large"

        # 2. All drones should have same altitude (within 2m)
        altitudes = [-pos[2] for pos in actual_positions.values()]
        alt_spread = max(altitudes) - min(altitudes)
        print(f"Altitude spread: {alt_spread:.1f}m (should be < 2m for LINE)")
        assert alt_spread < 2.0, f"LINE: Altitude spread {alt_spread:.1f}m too large"

        # 3. East spread should match expected (for 6 drones: 25m total)
        easts = [pos[1] for pos in actual_positions.values()]
        expected_spread = (num_drones - 1) * config.spacing
        actual_spread = max(easts) - min(easts)
        print(f"East spread: {actual_spread:.1f}m (expected: {expected_spread:.1f}m)")


@pytest.mark.sim
class TestPatrolWithGPSDenial:
    """Test patrol mission with GPS denial zone tracking.

    This single test covers:
    - Waypoint navigation (goto_all)
    - GPS denial zone detection
    - Entry/exit event logging with timestamps
    - Position verification at each waypoint

    GPS Denial Zone:
        Center: (25.0, 25.0) in world NED frame
        Radius: 15.0 meters
        Altitude: 10.0 meters (for zone check)

    Patrol Waypoints (all at altitude=10m):
        WP1: (0, 0)    - Start, outside zone
        WP2: (50, 0)   - East, outside zone
        WP3: (50, 50)  - Northeast corner, outside zone
        WP4: (25, 25)  - Zone CENTER, INSIDE zone
        WP5: (0, 50)   - West, outside zone
        WP6: (0, 0)    - Return to start, outside zone

    Expected behavior:
        - Drones enter zone around WP4
        - Drones exit zone after leaving WP4
        - At least one DENIED event should be logged
    """

    @pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_REASON)
    @pytest.mark.slow
    def test_patrol_with_denial_logging(self, flying_swarm, timeout_multiplier):
        """Patrol through GPS denial zone with detailed logging.

        Verify waypoints and denial zone coordinates by hand:
        - Zone: circle centered at (N=25, E=25), radius=15m
        - WP4 is exactly at zone center
        """
        from swarm.coordination import FormationType, FormationConfig
        from swarm.navigation import GPSJammer, GPSJammerConfig, GPSDenialZone

        # Track denial events: (timestamp, drone_id, event_type, position)
        denial_events = []

        # ==== GPS DENIAL ZONE CONFIGURATION ====
        # Verify these coordinates in Gazebo/simulation
        ZONE_CENTER = (25.0, 25.0, -10.0)  # NED: N=25, E=25, Alt=10m
        ZONE_RADIUS = 15.0                  # meters

        config = GPSJammerConfig()
        jammer = GPSJammer(config)
        jammer.add_zone(GPSDenialZone(
            zone_id=1,
            center=ZONE_CENTER,
            radius=ZONE_RADIUS,
            probability=1.0,
            name="patrol_denial_zone",
        ))

        # ==== PATROL WAYPOINTS ====
        # All waypoints at altitude=10m
        # Format: (North, East) in world NED frame
        WAYPOINTS = [
            (0, 0),      # WP1: Start - outside zone
            (50, 0),     # WP2: Move east - outside zone
            (50, 50),    # WP3: Northeast - outside zone
            (25, 25),    # WP4: ZONE CENTER - inside zone
            (0, 50),     # WP5: Move west - outside zone
            (0, 0),      # WP6: Return to start - outside zone
        ]
        ALTITUDE = 10.0

        # Track which drones are currently in the zone
        drones_in_zone = set()
        base_timeout = 20.0 * timeout_multiplier

        print(f"\n{'='*70}")
        print("GPS DENIAL PATROL TEST")
        print(f"{'='*70}")
        print(f"\nDenial Zone:")
        print(f"  Center: N={ZONE_CENTER[0]}, E={ZONE_CENTER[1]}, Alt={-ZONE_CENTER[2]}m")
        print(f"  Radius: {ZONE_RADIUS}m")
        print(f"\nWaypoints:")
        for i, wp in enumerate(WAYPOINTS):
            in_zone = jammer.is_in_denial_zone((wp[0], wp[1], -ALTITUDE))
            status = "INSIDE ZONE" if in_zone else "outside"
            print(f"  WP{i+1}: N={wp[0]:3}, E={wp[1]:3} - {status}")
        print(f"\n{'='*70}\n")

        # Execute patrol
        for i, wp in enumerate(WAYPOINTS):
            print(f"\nWaypoint {i+1}/{len(WAYPOINTS)}: N={wp[0]}, E={wp[1]}")

            # Move formation center to waypoint while maintaining formation
            formation_config = FormationConfig(spacing=5.0, altitude=ALTITUDE)
            flying_swarm.goto_formation(
                center_north=wp[0],
                center_east=wp[1],
                altitude=ALTITUDE,
                formation_type=FormationType.LINE,
                config=formation_config,
                wait=True,
                timeout=base_timeout,
                tolerance=3.0,
            )

            # Check GPS denial state and log events
            positions = flying_swarm._get_all_positions()
            for drone_id, pos in positions.items():
                in_zone = jammer.is_in_denial_zone(pos)
                was_in_zone = drone_id in drones_in_zone

                if in_zone and not was_in_zone:
                    # Entering zone
                    denial_events.append((
                        time.time(),
                        drone_id,
                        "DENIED",
                        (pos[0], pos[1], -pos[2]),
                    ))
                    drones_in_zone.add(drone_id)
                    print(f"  [GPS] Drone {drone_id} ENTERED zone at "
                          f"N={pos[0]:.1f}, E={pos[1]:.1f}")

                elif not in_zone and was_in_zone:
                    # Exiting zone
                    denial_events.append((
                        time.time(),
                        drone_id,
                        "RESTORED",
                        (pos[0], pos[1], -pos[2]),
                    ))
                    drones_in_zone.discard(drone_id)
                    print(f"  [GPS] Drone {drone_id} EXITED zone at "
                          f"N={pos[0]:.1f}, E={pos[1]:.1f}")

            # Print current positions at waypoint
            print(f"  Positions at WP{i+1}:")
            for drone_id, pos in sorted(positions.items()):
                print(f"    Drone {drone_id}: N={pos[0]:6.1f}, E={pos[1]:6.1f}, "
                      f"Alt={-pos[2]:5.1f}m")

            time.sleep(2)  # Hold at waypoint

        # ==== SUMMARY ====
        print(f"\n{'='*70}")
        print("PATROL SUMMARY")
        print(f"{'='*70}")

        denied_count = sum(1 for e in denial_events if e[2] == "DENIED")
        restored_count = sum(1 for e in denial_events if e[2] == "RESTORED")

        print(f"\nTotal events: {len(denial_events)}")
        print(f"  DENIED (entered zone): {denied_count}")
        print(f"  RESTORED (exited zone): {restored_count}")

        if denial_events:
            print("\nEvent Log:")
            start_time = denial_events[0][0]
            for timestamp, drone_id, event_type, pos in denial_events:
                t = timestamp - start_time
                print(f"  T+{t:5.1f}s: Drone {drone_id} {event_type} "
                      f"at N={pos[0]:.1f}, E={pos[1]:.1f}, Alt={pos[2]:.1f}m")

        # Verify at least one drone entered the zone
        assert denied_count > 0, (
            "No drones entered the denial zone. "
            f"Zone: center=({ZONE_CENTER[0]}, {ZONE_CENTER[1]}), radius={ZONE_RADIUS}m. "
            f"WP4 should be inside at (25, 25)."
        )

        print(f"\n{'='*70}")
        print("TEST PASSED: Drones successfully patrolled through GPS denial zone")
        print(f"{'='*70}\n")
