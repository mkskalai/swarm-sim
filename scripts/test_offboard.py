#!/usr/bin/env python3
"""Test script for offboard control mode.

This script demonstrates offboard control with position and velocity setpoints.

Usage:
    1. Start Gazebo with drone world:
       gz sim worlds/single_drone.sdf

    2. Start ArduPilot SITL in another terminal:
       sim_vehicle.py -v ArduCopter -f gazebo-iris --console

    3. Run this script:
       python scripts/test_offboard.py
"""

import asyncio
import logging
import sys
import math
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from swarm.core import Drone, DroneConfig

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


async def fly_square(drone: Drone, size: float = 10.0, altitude: float = 10.0):
    """Fly a square pattern using offboard position control.

    Args:
        drone: Connected drone instance.
        size: Side length of square in meters.
        altitude: Flight altitude in meters.
    """
    # Square corners (NED coordinates, down is negative for altitude)
    waypoints = [
        (0.0, 0.0, -altitude, 0.0),      # Start
        (size, 0.0, -altitude, 0.0),     # North
        (size, size, -altitude, 90.0),   # North-East
        (0.0, size, -altitude, 180.0),   # East
        (0.0, 0.0, -altitude, 270.0),    # Back to start
    ]

    logger.info(f"Flying square pattern: {size}m x {size}m at {altitude}m altitude")

    for i, (north, east, down, yaw) in enumerate(waypoints):
        logger.info(f"Waypoint {i + 1}/{len(waypoints)}: N={north}, E={east}")
        await drone.goto_position_ned(north, east, down, yaw, tolerance=1.0)
        await asyncio.sleep(2.0)  # Hover at each corner

    logger.info("Square pattern complete!")


async def fly_circle(drone: Drone, radius: float = 10.0, altitude: float = 10.0):
    """Fly a circular pattern using offboard velocity control.

    Args:
        drone: Connected drone instance.
        radius: Circle radius in meters.
        altitude: Flight altitude in meters.
    """
    logger.info(f"Flying circular pattern: {radius}m radius at {altitude}m altitude")

    # First move to starting position (east of center)
    await drone.goto_position_ned(0.0, radius, -altitude, 0.0, tolerance=1.0)

    # Fly circle using velocity commands
    angular_velocity = 0.2  # rad/s
    duration = 2 * math.pi / angular_velocity  # Time to complete circle
    dt = 0.05  # Control loop period

    start_time = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start_time < duration:
        t = asyncio.get_event_loop().time() - start_time
        angle = angular_velocity * t

        # Velocity tangent to circle
        vel_north = -radius * angular_velocity * math.sin(angle)
        vel_east = radius * angular_velocity * math.cos(angle)

        # Yaw facing direction of travel
        yaw = math.degrees(angle + math.pi / 2)

        await drone.set_velocity_ned(vel_north, vel_east, 0.0, yaw)
        await asyncio.sleep(dt)

    # Stop
    await drone.set_velocity_ned(0.0, 0.0, 0.0, 0.0)
    logger.info("Circle pattern complete!")


async def test_offboard_position():
    """Test offboard mode with position setpoints."""
    logger.info("=== Offboard Position Control Test ===")

    drone = Drone()

    if not await drone.connect():
        return False

    try:
        # Takeoff
        if not await drone.takeoff(altitude=10.0):
            return False

        await asyncio.sleep(3)

        # Start offboard mode
        if not await drone.start_offboard():
            logger.error("Failed to start offboard mode")
            return False

        # Fly square pattern
        await fly_square(drone, size=10.0, altitude=10.0)

        # Stop offboard
        await drone.stop_offboard()

        # Land
        await drone.land()
        await asyncio.sleep(2)
        await drone.disarm()

        return True

    finally:
        await drone.disconnect()


async def test_offboard_velocity():
    """Test offboard mode with velocity setpoints."""
    logger.info("=== Offboard Velocity Control Test ===")

    drone = Drone()

    if not await drone.connect():
        return False

    try:
        # Takeoff
        if not await drone.takeoff(altitude=15.0):
            return False

        await asyncio.sleep(3)

        # Start offboard mode
        if not await drone.start_offboard():
            logger.error("Failed to start offboard mode")
            return False

        # Fly circle pattern
        await fly_circle(drone, radius=8.0, altitude=15.0)

        # Return to center
        logger.info("Returning to center...")
        await drone.goto_position_ned(0.0, 0.0, -15.0, 0.0, tolerance=1.0)

        # Stop offboard
        await drone.stop_offboard()

        # Land
        await drone.land()
        await asyncio.sleep(2)
        await drone.disarm()

        return True

    finally:
        await drone.disconnect()


async def main():
    """Run offboard control tests."""
    print("\n" + "=" * 60)
    print("DRONE SWARM - Offboard Control Tests")
    print("=" * 60 + "\n")

    tests = [
        ("Offboard Position", test_offboard_position),
        ("Offboard Velocity", test_offboard_velocity),
    ]

    results = {}

    for name, test_func in tests:
        print(f"\n--- Running: {name} ---\n")

        response = input(f"Run {name} test? (y/n): ").strip().lower()
        if response != "y":
            logger.info(f"Skipping {name}")
            results[name] = None
            continue

        try:
            results[name] = await test_func()
        except Exception as e:
            logger.error(f"Test {name} failed with exception: {e}")
            results[name] = False

        await asyncio.sleep(5)

    # Summary
    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    for name, passed in results.items():
        if passed is None:
            status = "SKIP"
        elif passed:
            status = "PASS"
        else:
            status = "FAIL"
        print(f"  {name}: {status}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
