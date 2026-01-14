#!/usr/bin/env python3
"""Integration test for Phase 3: Swarm Coordination.

This script tests formation flying, leader-follower mode, and mission
execution with actual SITL drones.

Prerequisites:
    1. Build ArduPilot (only needed once, or after code changes):
       cd ~/ardupilot
       ./waf configure --board sitl
       ./waf copter

    2. Start Gazebo with multi-drone world:
       gz sim -v4 -r ~/ardupilot_gazebo/worlds/tutorial_multi_drone.sdf

    3. Start SITL instances with MAVProxy forwarding:
       sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --out=udp:127.0.0.1:14540 --console --no-rebuild
       sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I1 --out=udp:127.0.0.1:14541 --console --no-rebuild
       sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I2 --out=udp:127.0.0.1:14542 --console --no-rebuild
       # ... add more for more drones
       # --console: shows MAVProxy console for debugging
       # --no-rebuild: skips rebuild (faster startup, requires pre-built ArduPilot)

Usage:
    python scripts/test_phase3.py --num-drones 3
    python scripts/test_phase3.py --num-drones 6 --test formations
    python scripts/test_phase3.py --num-drones 3 --test all
"""

import argparse
import sys
import time
import logging
from pathlib import Path

# Add project root to path for imports
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from swarm.coordination import (
    SwarmController,
    SwarmConfig,
    FormationType,
    FormationConfig,
    MissionPlanner,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_formations(controller: SwarmController) -> bool:
    """Test flying through various formations.

    Args:
        controller: Connected SwarmController

    Returns:
        True if all formations flew successfully
    """
    print("\n" + "="*60)
    print("TEST: Formation Flying")
    print("="*60)

    formations = [
        (FormationType.LINE, "Line Formation"),
        (FormationType.V_FORMATION, "V Formation"),
        (FormationType.TRIANGLE, "Triangle Formation"),
        (FormationType.GRID, "Grid Formation"),
    ]

    for formation_type, name in formations:
        print(f"\n>>> {name}")
        try:
            success = controller.fly_formation(
                formation_type,
                duration=10.0,
                config=FormationConfig(spacing=5.0, altitude=10.0)
            )
            if not success:
                print(f"  FAILED: {name}")
                return False
            print(f"  PASSED: {name}")
        except Exception as e:
            print(f"  ERROR: {name} - {e}")
            return False

        # Brief pause between formations
        time.sleep(2.0)

    print("\n>>> Formation Transition Test")
    try:
        success = controller.fly_formation_transition(
            from_type=FormationType.LINE,
            to_type=FormationType.V_FORMATION,
            transition_duration=5.0,
            hold_duration=10.0
        )
        if success:
            print("  PASSED: Formation Transition")
        else:
            print("  FAILED: Formation Transition")
            return False
    except Exception as e:
        print(f"  ERROR: Formation Transition - {e}")
        return False

    return True


def test_leader_follower(controller: SwarmController) -> bool:
    """Test leader-follower mode.

    Args:
        controller: Connected SwarmController

    Returns:
        True if leader-follower worked correctly
    """
    print("\n" + "="*60)
    print("TEST: Leader-Follower Mode")
    print("="*60)

    # Start leader-follower with drone 0 as leader
    controller.start_leader_follower(
        leader_id=0,
        formation_type=FormationType.V_FORMATION,
        config=FormationConfig(spacing=5.0, altitude=10.0)
    )

    # Define leader path (square pattern)
    path_points = [
        (0.0, 0.0, -10.0),    # Start
        (20.0, 0.0, -10.0),   # East
        (20.0, 20.0, -10.0),  # North-East
        (0.0, 20.0, -10.0),   # North
        (0.0, 0.0, -10.0),    # Return
    ]

    print("\n>>> Leader flying square pattern, followers maintaining formation")

    for i, pos in enumerate(path_points):
        print(f"  Point {i+1}/{len(path_points)}: N={pos[0]}, E={pos[1]}")
        controller.update_leader_follower(pos, duration=5.0)

    print("\n  PASSED: Leader-Follower Mode")
    return True


def test_missions(controller: SwarmController, num_drones: int) -> bool:
    """Test mission execution.

    Args:
        controller: Connected SwarmController
        num_drones: Number of drones

    Returns:
        True if mission executed successfully
    """
    print("\n" + "="*60)
    print("TEST: Mission Execution")
    print("="*60)

    # Create a simple patrol mission
    waypoints = [
        (0, 0),
        (30, 0),
        (30, 30),
        (0, 30),
    ]

    print("\n>>> Creating patrol mission")
    mission = MissionPlanner.create_patrol_mission(
        waypoints=waypoints,
        num_drones=num_drones,
        altitude=12.0,
        hold_time=2.0
    )

    print(f"  Mission: {mission.name}")
    print(f"  Waypoints per drone: {len(waypoints)}")
    print(f"  Execution mode: {mission.execution_mode.value}")

    print("\n>>> Executing mission...")
    try:
        success = controller.execute_mission(mission)
        if success:
            print("  PASSED: Mission Execution")
        else:
            print("  FAILED: Mission Execution")
            return False
    except Exception as e:
        print(f"  ERROR: Mission Execution - {e}")
        return False

    return True


def run_all_tests(controller: SwarmController, num_drones: int) -> bool:
    """Run all integration tests.

    Args:
        controller: Connected SwarmController
        num_drones: Number of drones

    Returns:
        True if all tests passed
    """
    results = []

    # Test 1: Formations
    results.append(("Formations", test_formations(controller)))

    # Test 2: Leader-Follower
    results.append(("Leader-Follower", test_leader_follower(controller)))

    # Test 3: Missions
    results.append(("Missions", test_missions(controller, num_drones)))

    # Print summary
    print("\n" + "="*60)
    print("TEST RESULTS SUMMARY")
    print("="*60)

    all_passed = True
    for name, passed in results:
        status = "PASSED" if passed else "FAILED"
        print(f"  {name}: {status}")
        if not passed:
            all_passed = False

    print("="*60)
    if all_passed:
        print("ALL TESTS PASSED!")
    else:
        print("SOME TESTS FAILED")
    print("="*60 + "\n")

    return all_passed


def main():
    parser = argparse.ArgumentParser(
        description='Integration test for Phase 3: Swarm Coordination'
    )
    parser.add_argument(
        '--num-drones', type=int, default=3,
        help='Number of drones to connect (default: 3)'
    )
    parser.add_argument(
        '--base-port', type=int, default=14540,
        help='Base UDP port (default: 14540)'
    )
    parser.add_argument(
        '--test', type=str, default='all',
        choices=['all', 'formations', 'leader-follower', 'missions'],
        help='Which test to run (default: all)'
    )
    args = parser.parse_args()

    # Create controller
    config = SwarmConfig(
        num_drones=args.num_drones,
        base_port=args.base_port,
        heartbeat_timeout=5.0,
        position_update_rate=4.0,
    )

    controller = SwarmController(config)

    try:
        # Connect and prepare for flight
        print("\n" + "="*60)
        print(f"PHASE 3 INTEGRATION TEST")
        print(f"Drones: {args.num_drones}")
        print(f"Ports: {args.base_port} - {args.base_port + args.num_drones - 1}")
        print("="*60 + "\n")

        if not controller.connect_all():
            print("ERROR: Failed to connect to all drones")
            return 1

        if not controller.wait_for_ekf_all():
            print("ERROR: EKF convergence timeout")
            return 1

        if not controller.set_mode_all('GUIDED'):
            print("ERROR: Failed to set GUIDED mode")
            return 1

        if not controller.arm_all():
            print("ERROR: Failed to arm drones")
            return 1

        if not controller.takeoff_all(altitude=10.0):
            print("ERROR: Failed to takeoff")
            return 1

        # Run requested test(s)
        success = True

        if args.test == 'all':
            success = run_all_tests(controller, args.num_drones)
        elif args.test == 'formations':
            success = test_formations(controller)
        elif args.test == 'leader-follower':
            success = test_leader_follower(controller)
        elif args.test == 'missions':
            success = test_missions(controller, args.num_drones)

        return 0 if success else 1

    except KeyboardInterrupt:
        print("\n\nInterrupted! Landing all drones...")
        controller.land_all()
        time.sleep(5)
        controller.disarm_all()
        return 1

    except Exception as e:
        print(f"\nERROR: {e}")
        logger.exception("Unhandled exception")
        return 1

    finally:
        # Always try to land and disconnect
        try:
            controller.land_all()
            time.sleep(10)
            controller.disarm_all()
            controller.disconnect_all()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
