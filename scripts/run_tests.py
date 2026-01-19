#!/usr/bin/env python3
"""Unified test runner for drone swarm simulation.

This script provides a single entry point for running all tests:
- Unit tests: Fast Python tests, no simulation required
- Simulation tests: Full stack tests with Gazebo + SITL + ROS2

Usage:
    # Run all unit tests (fast, no simulation)
    python scripts/run_tests.py --unit

    # Run simulation tests (starts Gazebo + SITL + ROS2)
    python scripts/run_tests.py --sim -n 3

    # Run simulation with custom world
    python scripts/run_tests.py --sim -n 6 --world complex

    # Run simulation WITHOUT ROS2 (edge case for algorithm testing)
    python scripts/run_tests.py --sim -n 3 --no-ros

    # Skip simulation startup (assume already running)
    python scripts/run_tests.py --sim -n 3 --skip-sim

    # Run everything (unit + simulation)
    python scripts/run_tests.py --all -n 6

    # Increase timeouts for slow hardware (CPU-only or slow GPU)
    python scripts/run_tests.py --sim -n 3 --timeout-multiplier 3.0

    # Speed up simulation (2x real-time)
    python scripts/run_tests.py --sim -n 3 --speed 2.0

    # Run in Docker
    ./docker/scripts/run.sh test --sim -n 6

Timeout Note:
    If tests fail with timeout errors, use --timeout-multiplier to increase
    all timeout values. A value of 2.0 doubles timeouts, 3.0 triples them.
    CPU-only systems typically need 2-3x longer timeouts than GPU systems.

Simulation Speed Note:
    Use --speed to change simulation speed. 2.0 = 2x faster than real-time.
    Note: Faster speeds may cause physics instability.
"""

import argparse
import os
import signal
import sys
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def run_unit_tests(pytest_args: list) -> int:
    """Run unit tests only."""
    import pytest

    args = ["-v", "--tb=short", "-m", "unit", "tests/unit/"] + pytest_args
    print(f"\n{'='*60}")
    print("RUNNING UNIT TESTS")
    print(f"{'='*60}\n")

    return pytest.main(args)


def run_simulation_tests(
    num_drones: int,
    world: str,
    layout: str,
    headless: bool,
    use_ros: bool,
    skip_sim: bool,
    timeout_multiplier: float,
    sim_speed: float,
    pytest_args: list,
) -> int:
    """Run simulation tests with full stack."""
    import pytest
    from swarm.simulation import SimManager

    sim = None

    if not skip_sim:
        print(f"\n{'='*60}")
        print(f"STARTING SIMULATION ({num_drones} drones, {world} world, {layout} layout)")
        if sim_speed != 1.0:
            print(f"Simulation speed: {sim_speed}x real-time")
        print(f"{'='*60}\n")

        sim = SimManager(
            num_drones=num_drones,
            world=world,
            layout=layout,
            headless=headless,
            use_ros=use_ros,
            real_time_factor=sim_speed,
        )

        if not sim.start():
            print("ERROR: Failed to start simulation")
            return 1

    try:
        # Set environment variables for tests
        os.environ["SWARM_SIM_RUNNING"] = "1"
        os.environ["SWARM_NUM_DRONES"] = str(num_drones)
        os.environ["SWARM_TIMEOUT_MULTIPLIER"] = str(timeout_multiplier)

        if timeout_multiplier != 1.0:
            print(f"Timeout multiplier: {timeout_multiplier}x (all timeouts multiplied)")

        args = ["-v", "--tb=short", "-m", "sim", "tests/simulation/"] + pytest_args

        print(f"\n{'='*60}")
        print("RUNNING SIMULATION TESTS")
        print(f"{'='*60}\n")

        return pytest.main(args)

    finally:
        if sim:
            sim.stop()


def run_all_tests(
    num_drones: int,
    world: str,
    layout: str,
    headless: bool,
    use_ros: bool,
    skip_sim: bool,
    timeout_multiplier: float,
    sim_speed: float,
    pytest_args: list,
) -> int:
    """Run all tests (unit + simulation)."""
    import pytest
    from swarm.simulation import SimManager

    # Run unit tests first
    print(f"\n{'='*60}")
    print("PHASE 1: RUNNING UNIT TESTS")
    print(f"{'='*60}\n")

    unit_result = pytest.main(["-v", "--tb=short", "-m", "unit", "tests/unit/"] + pytest_args)

    if unit_result != 0:
        print("\nUnit tests failed, skipping simulation tests")
        return unit_result

    # Then run simulation tests
    print(f"\n{'='*60}")
    print("PHASE 2: RUNNING SIMULATION TESTS")
    if sim_speed != 1.0:
        print(f"Simulation speed: {sim_speed}x real-time")
    print(f"{'='*60}\n")

    sim = None
    if not skip_sim:
        sim = SimManager(
            num_drones=num_drones,
            world=world,
            layout=layout,
            headless=headless,
            use_ros=use_ros,
            real_time_factor=sim_speed,
        )

        if not sim.start():
            print("ERROR: Failed to start simulation")
            return 1

    try:
        os.environ["SWARM_SIM_RUNNING"] = "1"
        os.environ["SWARM_NUM_DRONES"] = str(num_drones)
        os.environ["SWARM_TIMEOUT_MULTIPLIER"] = str(timeout_multiplier)

        if timeout_multiplier != 1.0:
            print(f"Timeout multiplier: {timeout_multiplier}x (all timeouts multiplied)")

        return pytest.main(["-v", "--tb=short", "-m", "sim", "tests/simulation/"] + pytest_args)

    finally:
        if sim:
            sim.stop()


def main():
    parser = argparse.ArgumentParser(
        description="Unified test runner for drone swarm simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/run_tests.py --unit              # Unit tests only (fast)
    python scripts/run_tests.py --sim -n 3          # Simulation with 3 drones
    python scripts/run_tests.py --sim -n 6 --world complex
    python scripts/run_tests.py --all -n 6          # All tests
    python scripts/run_tests.py --sim -n 3 --no-ros # Without ROS2
        """,
    )

    # Test selection
    test_group = parser.add_mutually_exclusive_group()
    test_group.add_argument(
        "--unit",
        action="store_true",
        help="Run unit tests only (fast, no deps)",
    )
    test_group.add_argument(
        "--sim",
        action="store_true",
        help="Run simulation tests (ROS2 + Gazebo + SITL)",
    )
    test_group.add_argument(
        "--all",
        action="store_true",
        help="Run all tests (unit + simulation)",
    )

    # Simulation config
    parser.add_argument(
        "-n", "--num-drones",
        type=int,
        default=3,
        help="Number of drones (default: 3)",
    )
    parser.add_argument(
        "--world",
        choices=["basic", "complex"],
        default="complex",
        help="World type (default: complex)",
    )
    parser.add_argument(
        "--layout",
        choices=["grid", "line"],
        default="grid",
        help="Spawn layout (default: grid)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without Gazebo GUI",
    )
    parser.add_argument(
        "--skip-sim",
        action="store_true",
        help="Assume simulation already running",
    )
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="Skip ROS2 (algorithm testing only)",
    )

    # Timeout configuration
    parser.add_argument(
        "--timeout-multiplier",
        type=float,
        default=1.0,
        help="Multiply all timeouts by this factor (default: 1.0). "
             "Use 2.0-3.0 for CPU-only systems or slow hardware.",
    )

    # Simulation speed
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Simulation speed multiplier (default: 1.0 = real-time). "
             "Use 2.0 for 2x speed, 0.5 for half speed.",
    )

    # Pytest passthrough
    parser.add_argument(
        "pytest_args",
        nargs="*",
        help="Additional arguments passed to pytest",
    )

    args = parser.parse_args()

    # Default to unit tests if nothing specified
    if not (args.unit or args.sim or args.all):
        args.unit = True

    # Set up signal handlers
    def signal_handler(sig, frame):
        print("\nInterrupted!")
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Run appropriate tests
    if args.unit:
        return run_unit_tests(args.pytest_args)

    elif args.sim:
        return run_simulation_tests(
            num_drones=args.num_drones,
            world=args.world,
            layout=args.layout,
            headless=args.headless,
            use_ros=not args.no_ros,
            skip_sim=args.skip_sim,
            timeout_multiplier=args.timeout_multiplier,
            sim_speed=args.speed,
            pytest_args=args.pytest_args,
        )

    elif args.all:
        return run_all_tests(
            num_drones=args.num_drones,
            world=args.world,
            layout=args.layout,
            headless=args.headless,
            use_ros=not args.no_ros,
            skip_sim=args.skip_sim,
            timeout_multiplier=args.timeout_multiplier,
            sim_speed=args.speed,
            pytest_args=args.pytest_args,
        )


if __name__ == "__main__":
    sys.exit(main())
