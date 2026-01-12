#!/usr/bin/env python3
"""Test script for single drone connection and basic flight.

Usage:
    1. Start Gazebo with drone world:
       gz sim worlds/single_drone.sdf

    2. Start ArduPilot SITL in another terminal:
       sim_vehicle.py -v ArduCopter -f gazebo-iris --console

    3. Run this script:
       python scripts/test_connection.py
"""

import asyncio
import logging
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from swarm.core import Drone, DroneConfig

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


async def test_connection():
    """Test basic drone connection."""
    logger.info("=== Connection Test ===")

    config = DroneConfig()
    drone = Drone(config)

    if await drone.connect():
        logger.info(f"Connected! Drone state: {drone.state}")

        # Wait a moment for telemetry
        await asyncio.sleep(2)

        if drone.global_position:
            pos = drone.global_position
            logger.info(f"GPS Position: {pos.latitude:.6f}, {pos.longitude:.6f}, {pos.altitude:.1f}m")

        if drone.position:
            pos = drone.position
            logger.info(f"Local Position: N={pos.north:.1f}, E={pos.east:.1f}, D={pos.down:.1f}")

        logger.info(f"Battery: {drone.battery_percent:.1f}%")

        await drone.disconnect()
        return True
    else:
        logger.error("Connection failed!")
        return False


async def test_takeoff_land():
    """Test takeoff and landing."""
    logger.info("=== Takeoff/Land Test ===")

    drone = Drone()

    if not await drone.connect():
        return False

    try:
        # Takeoff
        if await drone.takeoff(altitude=5.0):
            logger.info("Hovering for 5 seconds...")
            await asyncio.sleep(5)

            # Land
            if await drone.land():
                logger.info("Landing successful!")

                # Disarm
                await asyncio.sleep(2)
                await drone.disarm()
                return True
    finally:
        await drone.disconnect()

    return False


async def test_goto():
    """Test goto position commands."""
    logger.info("=== Goto Position Test ===")

    drone = Drone()

    if not await drone.connect():
        return False

    try:
        if not await drone.takeoff(altitude=10.0):
            return False

        logger.info("Hovering for 3 seconds...")
        await asyncio.sleep(3)

        # Start offboard and move to position
        if await drone.start_offboard():
            logger.info("Moving north 10m...")
            await drone.goto_position_ned(10.0, 0.0, -10.0, 0.0, tolerance=1.0)

            logger.info("Moving east 10m...")
            await drone.goto_position_ned(10.0, 10.0, -10.0, 90.0, tolerance=1.0)

            logger.info("Returning to start...")
            await drone.goto_position_ned(0.0, 0.0, -10.0, 0.0, tolerance=1.0)

            await drone.stop_offboard()

        # Land
        await drone.land()
        await asyncio.sleep(2)
        await drone.disarm()
        return True

    finally:
        await drone.disconnect()


async def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("DRONE SWARM - Phase 1 Connection Tests")
    print("=" * 60 + "\n")

    tests = [
        ("Connection", test_connection),
        ("Takeoff/Land", test_takeoff_land),
        ("Goto Position", test_goto),
    ]

    results = {}

    for name, test_func in tests:
        print(f"\n--- Running: {name} ---\n")
        try:
            results[name] = await test_func()
        except Exception as e:
            logger.error(f"Test {name} failed with exception: {e}")
            results[name] = False

        # Pause between tests
        await asyncio.sleep(2)

    # Summary
    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
