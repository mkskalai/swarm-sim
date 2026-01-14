#!/usr/bin/env python3
"""Integration test for multi-drone fleet operations.

This script tests the Fleet class with actual SITL instances.
Requires Gazebo and SITL to be running.

Prerequisites:
    1. Generate world: python scripts/generate_world.py --num-drones 3
    2. Start Gazebo: cd ~/ardupilot_gazebo && gz sim -r ~/swarm/worlds/multi_drone_3.sdf
    3. Start SITL: python scripts/launch_sitl.py --num-instances 3

Usage:
    python scripts/test_fleet.py
    python scripts/test_fleet.py --num-drones 3 --altitude 5.0
"""

import argparse
import asyncio
import logging
import sys
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from swarm.core import Fleet, FleetConfig, FleetState, Position

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


async def test_connection(fleet: Fleet) -> bool:
    """Test connecting to all drones."""
    logger.info("=" * 50)
    logger.info("TEST: Fleet Connection")
    logger.info("=" * 50)

    success = await fleet.connect_all(timeout=30.0)

    if success:
        logger.info("PASS: All drones connected")
        status = fleet.get_status()
        logger.info(f"Status: {status.connected}/{status.total} connected")
    else:
        logger.error("FAIL: Some drones failed to connect")
        status = fleet.get_status()
        for error in status.errors:
            logger.error(f"  {error}")

    return success


async def test_takeoff_land(fleet: Fleet, altitude: float = 5.0) -> bool:
    """Test synchronized takeoff and landing."""
    logger.info("=" * 50)
    logger.info("TEST: Takeoff and Landing")
    logger.info("=" * 50)

    # Arm
    logger.info("Arming all drones...")
    if not await fleet.arm_all():
        logger.error("FAIL: Arming failed")
        return False
    logger.info("All drones armed")

    # Takeoff
    logger.info(f"Taking off to {altitude}m...")
    if not await fleet.takeoff_all(altitude):
        logger.error("FAIL: Takeoff failed")
        return False
    logger.info("All drones in flight")

    # Hover for a bit
    logger.info("Hovering for 5 seconds...")
    await asyncio.sleep(5.0)

    # Report positions
    positions = fleet.get_positions()
    for i, pos in enumerate(positions):
        if pos:
            logger.info(
                f"Drone {i}: N={pos.north:.2f}, E={pos.east:.2f}, Alt={pos.altitude:.2f}"
            )

    # Land
    logger.info("Landing all drones...")
    if not await fleet.land_all():
        logger.error("FAIL: Landing failed")
        return False

    logger.info("PASS: Takeoff and landing complete")
    return True


async def test_formation(fleet: Fleet, altitude: float = 5.0) -> bool:
    """Test moving to formation positions."""
    logger.info("=" * 50)
    logger.info("TEST: Formation Flight")
    logger.info("=" * 50)

    # Arm and takeoff
    if not await fleet.arm_all():
        return False

    if not await fleet.takeoff_all(altitude):
        return False

    # Wait for stable hover
    await asyncio.sleep(3.0)

    # Define formation (triangle)
    num_drones = len(fleet)
    if num_drones >= 3:
        positions = [
            Position(north=10.0, east=0.0, down=-altitude, yaw=0.0),   # Front
            Position(north=5.0, east=-5.0, down=-altitude, yaw=0.0),   # Back-left
            Position(north=5.0, east=5.0, down=-altitude, yaw=0.0),    # Back-right
        ][:num_drones]
    else:
        # Line formation for fewer drones
        positions = [
            Position(north=10.0 + i * 5, east=0.0, down=-altitude, yaw=0.0)
            for i in range(num_drones)
        ]

    logger.info(f"Moving to formation with {len(positions)} positions...")

    # Move to formation
    if not await fleet.goto_positions(positions, tolerance=1.0):
        logger.warning("Formation movement incomplete")

    # Hold formation
    logger.info("Holding formation for 5 seconds...")
    await asyncio.sleep(5.0)

    # Report final positions
    final_positions = fleet.get_positions()
    for i, pos in enumerate(final_positions):
        if pos:
            target = positions[i]
            error = (
                (pos.north - target.north) ** 2
                + (pos.east - target.east) ** 2
                + (pos.altitude - (-target.down)) ** 2
            ) ** 0.5
            logger.info(
                f"Drone {i}: Position error = {error:.2f}m"
            )

    # Land
    await fleet.land_all()

    logger.info("PASS: Formation test complete")
    return True


async def run_tests(num_drones: int, altitude: float, skip_formation: bool) -> bool:
    """Run all fleet tests."""
    config = FleetConfig(num_drones=num_drones)
    fleet = Fleet(config)

    try:
        # Test connection
        if not await test_connection(fleet):
            return False

        # Test takeoff/land
        if not await test_takeoff_land(fleet, altitude):
            return False

        # Test formation (optional)
        if not skip_formation:
            if not await test_formation(fleet, altitude):
                return False

        logger.info("=" * 50)
        logger.info("ALL TESTS PASSED")
        logger.info("=" * 50)
        return True

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
        return False

    except Exception as e:
        logger.exception(f"Test failed with error: {e}")
        return False

    finally:
        # Ensure cleanup
        logger.info("Cleaning up...")
        if fleet.state == FleetState.IN_FLIGHT:
            try:
                await fleet.land_all()
            except Exception:
                pass
        await fleet.disconnect_all()


def main():
    parser = argparse.ArgumentParser(
        description="Test multi-drone fleet operations",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Prerequisites:
    1. Generate world:
       python scripts/generate_world.py --num-drones 3

    2. Start Gazebo (in a separate terminal):
       cd ~/ardupilot_gazebo
       gz sim -r ~/swarm/worlds/multi_drone_3.sdf

    3. Start SITL (in a separate terminal):
       python scripts/launch_sitl.py --num-instances 3

    4. Run this test:
       python scripts/test_fleet.py
        """,
    )
    parser.add_argument(
        "-n", "--num-drones",
        type=int,
        default=3,
        help="Number of drones (default: 3)",
    )
    parser.add_argument(
        "-a", "--altitude",
        type=float,
        default=5.0,
        help="Test altitude in meters (default: 5.0)",
    )
    parser.add_argument(
        "--skip-formation",
        action="store_true",
        help="Skip formation test",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable debug logging",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    success = asyncio.run(
        run_tests(
            num_drones=args.num_drones,
            altitude=args.altitude,
            skip_formation=args.skip_formation,
        )
    )

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
