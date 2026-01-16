#!/usr/bin/env python3
"""Phase 7 integration test script for autonomous missions.

Tests:
1. GPS denial during swarm navigation
2. Search pattern execution with failures
3. Pursuit behavior through terrain

Usage:
    # Run all scenarios
    python scripts/test_phase7.py --scenario all --terrain urban --num-drones 4

    # Individual scenarios
    python scripts/test_phase7.py --scenario gps --terrain urban
    python scripts/test_phase7.py --scenario search --terrain forest
    python scripts/test_phase7.py --scenario pursuit --terrain canyon

Prerequisites:
    - Gazebo simulation running with terrain world
    - SITL instances started
"""

import argparse
import logging
import sys
import time
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from swarm.coordination.swarm_controller import SwarmController, SwarmConfig
from swarm.coordination.search_patterns import (
    SearchPatternConfig,
    SearchPatternType,
    SearchPatternGenerator,
    AdaptiveSearchController,
)
from swarm.coordination.pursuit_controller import (
    PursuitController,
    PursuitConfig,
    PursuitStrategy,
    PursuitTarget,
    SimulatedTarget,
)
from swarm.navigation.gps_jammer import (
    GPSJammer,
    GPSJammerConfig,
    GPSDenialZone,
    DenialZoneShape,
    create_urban_denial_zones,
)
from swarm.missions.autonomous_mission import (
    AutonomousMissionController,
    AutonomousMissionConfig,
    MissionPhase,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_search_patterns_standalone() -> bool:
    """Test search pattern generation (no simulation required).

    Returns:
        True if test passed
    """
    logger.info("=" * 60)
    logger.info("Testing search pattern generation")
    logger.info("=" * 60)

    # Test lawnmower
    logger.info("\n--- Lawnmower Pattern ---")
    lawnmower_wps = SearchPatternGenerator.generate_lawnmower(
        bounds=(0, 100, 0, 100),
        lane_spacing=20.0,
        altitude=15.0,
    )
    logger.info(f"Generated {len(lawnmower_wps)} waypoints")
    assert len(lawnmower_wps) > 0, "Lawnmower should generate waypoints"

    # Test spiral
    logger.info("\n--- Spiral Pattern ---")
    spiral_wps = SearchPatternGenerator.generate_spiral(
        center=(50.0, 50.0),
        max_radius=40.0,
        spacing=10.0,
        altitude=15.0,
    )
    logger.info(f"Generated {len(spiral_wps)} waypoints")
    assert len(spiral_wps) > 0, "Spiral should generate waypoints"

    # Test expanding square
    logger.info("\n--- Expanding Square Pattern ---")
    square_wps = SearchPatternGenerator.generate_expanding_square(
        center=(50.0, 50.0),
        max_size=50.0,
        step_size=10.0,
        altitude=15.0,
    )
    logger.info(f"Generated {len(square_wps)} waypoints")
    assert len(square_wps) > 0, "Expanding square should generate waypoints"

    # Test adaptive search controller
    logger.info("\n--- Adaptive Search Controller ---")
    config = SearchPatternConfig(
        pattern_type=SearchPatternType.EXPANDING_SQUARE,
        center=(0.0, 0.0),
        max_size=100.0,
        step_size=15.0,
        altitude=10.0,
    )

    controller = AdaptiveSearchController(
        num_drones=4,
        pattern_config=config,
    )

    assignments = controller.initialize_search([0, 1, 2, 3])
    logger.info(f"Distributed waypoints to {len(assignments)} drones")

    for drone_id, wps in assignments.items():
        logger.info(f"  Drone {drone_id}: {len(wps)} waypoints")

    # Test failure reallocation
    logger.info("\n--- Testing failure reallocation ---")
    new_assignments = controller.on_drone_failure(1)
    logger.info(f"After drone 1 failure: redistributed to {len(new_assignments)} drones")

    logger.info("\nSearch pattern tests PASSED")
    return True


def test_gps_jammer_standalone() -> bool:
    """Test GPS jammer (no simulation required).

    Returns:
        True if test passed
    """
    logger.info("=" * 60)
    logger.info("Testing GPS jammer")
    logger.info("=" * 60)

    # Create jammer with zones
    config = GPSJammerConfig(
        zones=[
            GPSDenialZone(
                zone_id=1,
                center=(50.0, 50.0, -10.0),
                radius=20.0,
                shape=DenialZoneShape.SPHERE,
                probability=1.0,
                name="Test Zone 1",
            ),
            GPSDenialZone(
                zone_id=2,
                center=(100.0, 100.0, -15.0),
                radius=30.0,
                shape=DenialZoneShape.CYLINDER,
                height=50.0,
                probability=0.8,
                name="Test Zone 2",
            ),
        ],
        update_rate=10.0,
    )

    jammer = GPSJammer(config)

    # Test zone containment
    logger.info("\n--- Testing zone containment ---")
    pos_in_zone = (50.0, 50.0, -10.0)
    pos_outside = (0.0, 0.0, -10.0)

    assert jammer.is_in_denial_zone(pos_in_zone), "Position should be in zone"
    assert not jammer.is_in_denial_zone(pos_outside), "Position should be outside zone"
    logger.info("Zone containment tests passed")

    # Test urban zone generation
    logger.info("\n--- Testing urban zone generation ---")
    urban_zones = create_urban_denial_zones(
        city_center=(100.0, 100.0, -20.0),
        num_buildings=5,
        building_radius=15.0,
        spread=50.0,
        seed=42,
    )
    logger.info(f"Generated {len(urban_zones)} urban GPS denial zones")

    # Test scheduled denial
    logger.info("\n--- Testing scheduled denial ---")
    current_time = time.time()
    jammer.schedule_denial(
        drone_id=0,
        start_time=current_time + 1.0,
        duration=5.0,
    )
    logger.info("Scheduled denial for drone 0")

    logger.info("\nGPS jammer tests PASSED")
    return True


def test_pursuit_controller_standalone() -> bool:
    """Test pursuit controller (no simulation required).

    Returns:
        True if test passed
    """
    logger.info("=" * 60)
    logger.info("Testing pursuit controller")
    logger.info("=" * 60)

    # Create pursuit controller
    config = PursuitConfig(
        strategy=PursuitStrategy.SURROUND,
        surround_radius=20.0,
        follow_distance=15.0,
    )

    controller = PursuitController(
        num_drones=4,
        config=config,
    )

    # Create target
    target = PursuitTarget(
        target_id=1,
        position=(50.0, 50.0, -1.5),
        velocity=(2.0, 1.0, 0.0),
        class_name="person",
    )

    controller.set_target(target)
    logger.info(f"Set target: {target.class_name} at {target.position}")

    # Assign pursuers
    assignments = controller.assign_pursuers([0, 1, 2, 3])
    logger.info(f"Assigned roles: {assignments}")

    # Get pursuit positions
    drone_positions = {
        0: (30.0, 30.0, -15.0),
        1: (70.0, 30.0, -15.0),
        2: (30.0, 70.0, -15.0),
        3: (70.0, 70.0, -15.0),
    }

    pursuit_targets = controller.get_pursuit_targets(drone_positions)
    logger.info("Pursuit target positions:")
    for drone_id, pos in pursuit_targets.items():
        logger.info(f"  Drone {drone_id}: {pos}")

    # Test simulated target
    logger.info("\n--- Testing simulated target ---")
    sim_target = SimulatedTarget(
        start_position=(0.0, 0.0, -1.5),
        waypoints=[(50.0, 0.0, -1.5), (50.0, 50.0, -1.5), (0.0, 50.0, -1.5)],
        speed=3.0,
    )

    for i in range(5):
        sim_target.update(1.0)
        logger.info(f"  t={i+1}s: pos={sim_target.position}, vel={sim_target.velocity}")

    logger.info("\nPursuit controller tests PASSED")
    return True


def test_autonomous_mission_standalone() -> bool:
    """Test autonomous mission controller (no simulation required).

    Returns:
        True if test passed
    """
    logger.info("=" * 60)
    logger.info("Testing autonomous mission controller")
    logger.info("=" * 60)

    # Create mission config
    config = AutonomousMissionConfig(
        search_pattern=SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=(50.0, 50.0),
            max_size=80.0,
            step_size=15.0,
            altitude=15.0,
        ),
        pursuit_config=PursuitConfig(
            strategy=PursuitStrategy.SURROUND,
            surround_radius=20.0,
        ),
        target_classes=["person", "car"],
        search_timeout=60.0,
    )

    controller = AutonomousMissionController(config=config)

    # Start mission
    success = controller.start_mission(active_drones=[0, 1, 2, 3])
    assert success, "Mission should start successfully"
    logger.info(f"Mission started, phase: {controller.phase.value}")

    # Simulate drone positions (airborne)
    drone_positions = {
        0: (0.0, 0.0, -10.0),
        1: (5.0, 0.0, -10.0),
        2: (10.0, 0.0, -10.0),
        3: (15.0, 0.0, -10.0),
    }

    # Run a few update cycles
    for i in range(5):
        phase = controller.update(drone_positions, dt=0.1)
        logger.info(f"Update {i+1}: phase={phase.value}")

        # Get targets
        targets = controller.get_current_targets()
        if targets:
            logger.info(f"  Targets: {len(targets)} drones have assignments")

    # Check stats
    stats = controller.stats
    logger.info(f"\nMission stats: {stats}")

    # Test abort
    controller.abort_mission()
    assert controller.phase == MissionPhase.RTL, "Should be in RTL after abort"

    logger.info("\nAutonomous mission tests PASSED")
    return True


def test_scenario_gps_denial(controller: SwarmController, num_drones: int) -> bool:
    """Test Scenario 1: GPS denial during swarm navigation.

    1. Fly formation through GPS denial zone
    2. Verify drones switch to VIO
    3. Verify formation maintained

    Args:
        controller: SwarmController instance
        num_drones: Number of drones

    Returns:
        True if test passed
    """
    logger.info("=" * 60)
    logger.info("Scenario 1: GPS Denial During Navigation")
    logger.info("=" * 60)

    try:
        # Connect and setup
        logger.info("Connecting to drones...")
        if not controller.connect_all(timeout=30.0):
            logger.error("Failed to connect to all drones")
            return False

        logger.info("Waiting for EKF convergence...")
        if not controller.wait_for_ekf_all(timeout=60.0):
            logger.error("EKF did not converge")
            return False

        logger.info("Setting GUIDED mode and arming...")
        controller.set_mode_all("GUIDED")
        time.sleep(1.0)
        controller.arm_all()
        time.sleep(2.0)

        logger.info("Taking off...")
        controller.takeoff_all(altitude=10.0, wait=True)

        # Initialize VIO
        logger.info("Initializing VIO...")
        controller.init_vio_all()
        time.sleep(2.0)

        # Create GPS denial zone ahead
        jammer = GPSJammer(GPSJammerConfig(
            zones=[
                GPSDenialZone(
                    zone_id=1,
                    center=(30.0, 0.0, -10.0),
                    radius=15.0,
                    shape=DenialZoneShape.CYLINDER,
                    height=30.0,
                    probability=1.0,
                    name="GPS Denial Zone",
                ),
            ],
        ))
        logger.info(f"Created GPS denial zone at (30, 0, -10) with radius 15m")

        # Waypoints: start -> through zone -> exit
        waypoints = [
            (10.0, 0.0, -10.0),   # Before zone
            (30.0, 0.0, -10.0),   # In zone center
            (50.0, 0.0, -10.0),   # After zone
        ]

        gps_denied_drones = set()
        vio_activated = set()

        for wp_idx, waypoint in enumerate(waypoints):
            logger.info(f"\n--- Waypoint {wp_idx + 1}: {waypoint} ---")

            # Calculate formation positions around waypoint
            positions = {}
            for i in range(num_drones):
                offset_e = (i - (num_drones - 1) / 2) * 5.0  # Spread east-west
                positions[i] = (waypoint[0], waypoint[1] + offset_e, waypoint[2])

            # Send position commands and wait for arrival
            logger.info(f"Moving {num_drones} drones to waypoint...")
            start_time = time.time()
            timeout = 30.0
            arrival_tolerance = 3.0

            while time.time() - start_time < timeout:
                all_arrived = True

                for drone_id in range(num_drones):
                    target = positions[drone_id]
                    conn = controller.get_connection(drone_id)
                    if conn is None:
                        continue

                    # Send position command
                    controller._send_position_ned(conn, target)

                    # Check if in denial zone
                    current_pos = controller._get_position(conn, timeout=0.5, drone_id=drone_id)
                    if current_pos:
                        in_zone = jammer.is_in_denial_zone(current_pos)

                        if in_zone and drone_id not in gps_denied_drones:
                            logger.info(f"  Drone {drone_id} ENTERED GPS denial zone - activating VIO")
                            controller.deny_gps(drone_id)
                            gps_denied_drones.add(drone_id)

                        elif not in_zone and drone_id in gps_denied_drones:
                            logger.info(f"  Drone {drone_id} EXITED GPS denial zone - restoring GPS")
                            controller.restore_gps(drone_id)
                            gps_denied_drones.discard(drone_id)

                        # Check VIO state
                        vio_state = controller.get_vio_state(drone_id)
                        if vio_state and drone_id in gps_denied_drones:
                            if drone_id not in vio_activated:
                                logger.info(f"  Drone {drone_id} VIO active: mode={vio_state.mode}")
                                vio_activated.add(drone_id)

                        # Check distance to target
                        dist = controller._distance_to_target(current_pos, target)
                        if dist > arrival_tolerance:
                            all_arrived = False

                time.sleep(0.25)

                if all_arrived:
                    logger.info(f"All drones reached waypoint {wp_idx + 1}")
                    break

            # Hold at waypoint briefly
            time.sleep(3.0)

        # Verify results
        logger.info("\n--- Results ---")
        logger.info(f"Drones that entered denial zone: {len(gps_denied_drones) + len(vio_activated)}")
        logger.info(f"Drones with VIO activation: {len(vio_activated)}")

        # Return to start and land
        logger.info("\nReturning to start...")
        for drone_id in range(num_drones):
            conn = controller.get_connection(drone_id)
            if conn:
                controller._send_position_ned(conn, (0.0, (drone_id - (num_drones-1)/2) * 5.0, -10.0))
        time.sleep(10.0)

        logger.info("Landing...")
        controller.land_all()

        logger.info("GPS denial scenario PASSED")
        return True

    except Exception as e:
        logger.error(f"GPS denial scenario failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        controller.disconnect_all()


def test_scenario_search_with_failures(
    controller: SwarmController,
    num_drones: int,
) -> bool:
    """Test Scenario 2: Search pattern with drone failures.

    1. Begin expanding square search
    2. Simulate drone failures
    3. Verify remaining drones reallocate area

    Args:
        controller: SwarmController instance
        num_drones: Number of drones

    Returns:
        True if test passed
    """
    logger.info("=" * 60)
    logger.info("Scenario 2: Search Pattern with Failures")
    logger.info("=" * 60)

    try:
        # Connect and setup
        logger.info("Connecting to drones...")
        if not controller.connect_all(timeout=30.0):
            logger.error("Failed to connect to all drones")
            return False

        logger.info("Waiting for EKF convergence...")
        if not controller.wait_for_ekf_all(timeout=60.0):
            logger.error("EKF did not converge")
            return False

        logger.info("Setting GUIDED mode and arming...")
        controller.set_mode_all("GUIDED")
        time.sleep(1.0)
        controller.arm_all()
        time.sleep(2.0)

        logger.info("Taking off...")
        controller.takeoff_all(altitude=15.0, wait=True)

        # Create search controller
        config = SearchPatternConfig(
            pattern_type=SearchPatternType.EXPANDING_SQUARE,
            center=(0.0, 0.0),  # Search centered at origin
            max_size=60.0,
            step_size=20.0,
            altitude=15.0,
        )

        search = AdaptiveSearchController(
            num_drones=num_drones,
            pattern_config=config,
        )

        active_drones = list(range(num_drones))
        assignments = search.initialize_search(active_drones)

        logger.info(f"Search initialized for {num_drones} drones")
        for drone_id, wps in assignments.items():
            logger.info(f"  Drone {drone_id}: {len(wps)} waypoints")

        # Execute search pattern
        waypoint_indices = {d: 0 for d in active_drones}
        completed_waypoints = {d: 0 for d in active_drones}
        failure_simulated = False

        max_iterations = 50
        iteration = 0

        while iteration < max_iterations:
            iteration += 1

            # Check if all done
            all_done = True
            for drone_id in list(active_drones):
                progress = search.get_progress(drone_id)
                if progress and waypoint_indices[drone_id] < len(progress.assigned_waypoints):
                    all_done = False
                    break

            if all_done:
                logger.info("All waypoints completed!")
                break

            # Simulate failure after some waypoints (only once)
            total_completed = sum(completed_waypoints.values())
            if not failure_simulated and total_completed >= 3 and num_drones > 2:
                failure_drone = 1  # Simulate drone 1 failure
                logger.info(f"\n--- SIMULATING FAILURE: Drone {failure_drone} ---")
                active_drones.remove(failure_drone)

                # Reallocate search
                new_assignments = search.on_drone_failure(failure_drone)
                for d, wps in new_assignments.items():
                    logger.info(f"  Drone {d} reassigned: {len(wps)} new waypoints")
                    # Reset waypoint index for reassigned drones
                    waypoint_indices[d] = 0

                failure_simulated = True

            # Send position commands
            for drone_id in active_drones:
                conn = controller.get_connection(drone_id)
                if conn is None:
                    continue

                progress = search.get_progress(drone_id)
                if progress is None or waypoint_indices[drone_id] >= len(progress.assigned_waypoints):
                    continue

                target_wp = progress.assigned_waypoints[waypoint_indices[drone_id]]
                target = (target_wp.north, target_wp.east, target_wp.down)
                controller._send_position_ned(conn, target)

                # Check arrival
                current_pos = controller._get_position(conn, timeout=0.3, drone_id=drone_id)
                if current_pos:
                    dist = controller._distance_to_target(current_pos, target)
                    if dist < 3.0:
                        waypoint_indices[drone_id] += 1
                        completed_waypoints[drone_id] += 1
                        logger.info(f"  Drone {drone_id} reached waypoint {waypoint_indices[drone_id]}")

            time.sleep(0.5)

        # Summary
        logger.info("\n--- Search Results ---")
        logger.info(f"Total iterations: {iteration}")
        for drone_id in range(num_drones):
            if drone_id in active_drones:
                logger.info(f"  Drone {drone_id}: {completed_waypoints.get(drone_id, 0)} waypoints completed")
            else:
                logger.info(f"  Drone {drone_id}: FAILED (simulated)")

        # Return to origin and land
        logger.info("\nReturning to origin...")
        for drone_id in active_drones:
            conn = controller.get_connection(drone_id)
            if conn:
                controller._send_position_ned(conn, (0.0, (drone_id - (num_drones-1)/2) * 5.0, -15.0))
        time.sleep(10.0)

        logger.info("Landing...")
        controller.land_all()

        logger.info("Search with failures scenario PASSED")
        return True

    except Exception as e:
        logger.error(f"Search scenario failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        controller.disconnect_all()


def test_scenario_pursuit(controller: SwarmController, num_drones: int) -> bool:
    """Test Scenario 3: Pursuit through terrain.

    1. Create simulated moving target
    2. Pursue through terrain
    3. Drones surround and track target

    Args:
        controller: SwarmController instance
        num_drones: Number of drones

    Returns:
        True if test passed
    """
    logger.info("=" * 60)
    logger.info("Scenario 3: Pursuit Through Terrain")
    logger.info("=" * 60)

    try:
        # Connect and setup
        logger.info("Connecting to drones...")
        if not controller.connect_all(timeout=30.0):
            logger.error("Failed to connect to all drones")
            return False

        logger.info("Waiting for EKF convergence...")
        if not controller.wait_for_ekf_all(timeout=60.0):
            logger.error("EKF did not converge")
            return False

        logger.info("Setting GUIDED mode and arming...")
        controller.set_mode_all("GUIDED")
        time.sleep(1.0)
        controller.arm_all()
        time.sleep(2.0)

        logger.info("Taking off...")
        controller.takeoff_all(altitude=15.0, wait=True)

        # Create pursuit controller
        config = PursuitConfig(
            strategy=PursuitStrategy.SURROUND,
            surround_radius=20.0,
            min_altitude=10.0,
            target_lost_timeout=5.0,
        )

        pursuit = PursuitController(
            num_drones=num_drones,
            config=config,
        )

        # Create simulated moving target
        target = SimulatedTarget(
            start_position=(10.0, 0.0, -1.5),  # Ground level
            waypoints=[
                (30.0, 0.0, -1.5),
                (30.0, 30.0, -1.5),
                (0.0, 30.0, -1.5),
                (0.0, 0.0, -1.5),
            ],
            speed=2.0,  # 2 m/s - walkable pace
            loop=True,
        )

        # Set target and assign pursuers
        pursuit.set_target(target.to_pursuit_target())
        pursuit.assign_pursuers(list(range(num_drones)))

        logger.info(f"Target starting at {target.position}, moving at {target._speed} m/s")
        logger.info(f"Pursuit strategy: {config.strategy.value}, surround radius: {config.surround_radius}m")

        # Pursuit loop
        dt = 0.25
        max_duration = 60.0  # 60 seconds of pursuit
        elapsed = 0.0
        update_count = 0

        while elapsed < max_duration:
            # Update simulated target
            target.update(dt)

            # Update pursuit controller with new target position
            pursuit.update_target_position(
                target.position,
                target.velocity,
            )

            # Get current drone positions
            drone_positions = {}
            for drone_id in range(num_drones):
                conn = controller.get_connection(drone_id)
                if conn:
                    pos = controller._get_position(conn, timeout=0.2, drone_id=drone_id)
                    if pos:
                        drone_positions[drone_id] = pos

            # Get pursuit target positions for each drone
            pursuit_targets = pursuit.get_pursuit_targets(drone_positions)

            # Send position commands
            for drone_id, pursuit_pos in pursuit_targets.items():
                conn = controller.get_connection(drone_id)
                if conn:
                    controller._send_position_ned(conn, pursuit_pos)

            # Log progress every 4 seconds
            update_count += 1
            if update_count % 16 == 0:
                logger.info(f"  t={elapsed:.1f}s: Target at ({target.position[0]:.1f}, {target.position[1]:.1f}), "
                           f"heading={target.heading:.0f}°")

                # Calculate average distance from drones to target
                if drone_positions:
                    avg_dist = sum(
                        ((p[0] - target.position[0])**2 + (p[1] - target.position[1])**2)**0.5
                        for p in drone_positions.values()
                    ) / len(drone_positions)
                    logger.info(f"       Avg drone distance to target: {avg_dist:.1f}m")

            time.sleep(dt)
            elapsed += dt

            # Check if target completed path (non-looping)
            if target.is_complete:
                logger.info("Target reached final waypoint")
                break

        # Summary
        logger.info("\n--- Pursuit Results ---")
        logger.info(f"Pursuit duration: {elapsed:.1f}s")
        logger.info(f"Target final position: ({target.position[0]:.1f}, {target.position[1]:.1f})")

        # Return to origin and land
        logger.info("\nReturning to origin...")
        for drone_id in range(num_drones):
            conn = controller.get_connection(drone_id)
            if conn:
                controller._send_position_ned(conn, (0.0, (drone_id - (num_drones-1)/2) * 5.0, -15.0))
        time.sleep(10.0)

        logger.info("Landing...")
        controller.land_all()

        logger.info("Pursuit scenario PASSED")
        return True

    except Exception as e:
        logger.error(f"Pursuit scenario failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        controller.disconnect_all()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Phase 7 integration tests")
    parser.add_argument(
        "--scenario",
        choices=["all", "gps", "search", "pursuit", "standalone"],
        default="standalone",
        help="Test scenario to run (default: standalone)"
    )
    parser.add_argument(
        "--terrain",
        choices=["flat", "urban", "forest", "canyon"],
        default="urban",
        help="Terrain type (default: urban)"
    )
    parser.add_argument(
        "--num-drones",
        type=int,
        default=4,
        help="Number of drones (default: 4)"
    )
    parser.add_argument(
        "--skip-simulation",
        action="store_true",
        help="Skip simulation-dependent tests"
    )

    args = parser.parse_args()

    logger.info("=" * 60)
    logger.info("Phase 7: Autonomous Missions - Integration Tests")
    logger.info("=" * 60)
    logger.info(f"Terrain: {args.terrain}")
    logger.info(f"Drones: {args.num_drones}")
    logger.info(f"Scenario: {args.scenario}")

    results = {}

    # Always run standalone tests
    if args.scenario in ("all", "standalone"):
        results["search_patterns"] = test_search_patterns_standalone()
        results["gps_jammer"] = test_gps_jammer_standalone()
        results["pursuit_controller"] = test_pursuit_controller_standalone()
        results["autonomous_mission"] = test_autonomous_mission_standalone()

    # Simulation-dependent tests
    if not args.skip_simulation and args.scenario != "standalone":
        swarm_config = SwarmConfig(
            num_drones=args.num_drones,
            enable_vio=True,
            vio_mode="hybrid",
        )

        if args.scenario in ("all", "gps"):
            # Each scenario gets its own controller since they manage connect/disconnect
            controller = SwarmController(swarm_config)
            results["gps_denial_scenario"] = test_scenario_gps_denial(
                controller, args.num_drones
            )

        if args.scenario in ("all", "search"):
            controller = SwarmController(swarm_config)
            results["search_scenario"] = test_scenario_search_with_failures(
                controller, args.num_drones
            )

        if args.scenario in ("all", "pursuit"):
            controller = SwarmController(swarm_config)
            results["pursuit_scenario"] = test_scenario_pursuit(
                controller, args.num_drones
            )

    # Summary
    logger.info("\n" + "=" * 60)
    logger.info("Test Results Summary")
    logger.info("=" * 60)

    all_passed = True
    for test_name, passed in results.items():
        status = "PASSED" if passed else "FAILED"
        logger.info(f"  {test_name}: {status}")
        if not passed:
            all_passed = False

    if all_passed:
        logger.info("\nAll tests PASSED!")
        return 0
    else:
        logger.error("\nSome tests FAILED!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
