#!/usr/bin/env python3
"""Phase 6 test script: GPS-Denied Navigation with VIO.

Tests visual-inertial odometry integration with the swarm controller.

Usage:
    python scripts/test_phase6.py --num-drones 3
    python scripts/test_phase6.py --num-drones 3 --gps-denial-test
"""

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from swarm.coordination import SwarmController, SwarmConfig
from swarm.coordination.formations import FormationType

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


def test_vio_initialization(controller: SwarmController) -> bool:
    """Test VIO initialization for all drones."""
    print("\n" + "=" * 60)
    print("TEST: VIO Initialization")
    print("=" * 60)

    success = controller.init_vio_all()

    if success:
        print("[PASS] VIO initialized for all drones")

        # Check VIO state for each drone
        for drone_id in range(controller.config.num_drones):
            state = controller.get_vio_state(drone_id)
            if state:
                print(f"  Drone {drone_id}:")
                print(f"    Position: ({state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f})")
                print(f"    Mode: {state.mode.name}")
                print(f"    VO Quality: {state.vo_quality:.2f}")
                print(f"    Scale Confidence: {state.scale_confidence:.2f}")
    else:
        print("[FAIL] VIO initialization failed for some drones")

    return success


def test_gps_denial(controller: SwarmController, drone_id: int = 0) -> bool:
    """Test GPS denial and VIO fallback."""
    print("\n" + "=" * 60)
    print(f"TEST: GPS Denial for Drone {drone_id}")
    print("=" * 60)

    # Get initial position
    state_before = controller.get_vio_state(drone_id)
    if not state_before:
        print("[FAIL] Could not get VIO state")
        return False

    print(f"Before GPS denial:")
    print(f"  Mode: {state_before.mode.name}")
    print(f"  Position: ({state_before.position[0]:.2f}, {state_before.position[1]:.2f}, {state_before.position[2]:.2f})")

    # Deny GPS
    controller.deny_gps(drone_id)
    print("\nGPS denied. Waiting 5 seconds...")
    time.sleep(5.0)

    # Check mode switched to VIO
    state_after = controller.get_vio_state(drone_id)
    if state_after:
        print(f"\nAfter GPS denial:")
        print(f"  Mode: {state_after.mode.name}")
        print(f"  Position: ({state_after.position[0]:.2f}, {state_after.position[1]:.2f}, {state_after.position[2]:.2f})")
        print(f"  VO Quality: {state_after.vo_quality:.2f}")

        from swarm.navigation import NavigationMode
        if state_after.mode in [NavigationMode.VIO, NavigationMode.DEAD_RECKONING]:
            print("\n[PASS] Switched to VIO/Dead Reckoning mode")
        else:
            print(f"\n[WARN] Still in {state_after.mode.name} mode")

    # Restore GPS
    controller.restore_gps(drone_id)
    print("\nGPS restored. Waiting 3 seconds...")
    time.sleep(3.0)

    state_restored = controller.get_vio_state(drone_id)
    if state_restored:
        print(f"\nAfter GPS restore:")
        print(f"  Mode: {state_restored.mode.name}")
        print(f"  Position: ({state_restored.position[0]:.2f}, {state_restored.position[1]:.2f}, {state_restored.position[2]:.2f})")

    return True


def test_formation_with_vio(controller: SwarmController) -> bool:
    """Test formation flying with VIO enabled."""
    print("\n" + "=" * 60)
    print("TEST: Formation Flying with VIO")
    print("=" * 60)

    # Fly V formation
    print("\nFlying V formation for 20 seconds...")
    success = controller.fly_formation(
        FormationType.V_FORMATION,
        center=(0, 0, -15),  # 15m altitude
        spacing=10.0,
        heading=0.0,
        duration=20.0,
        arrival_timeout=30.0,
        arrival_tolerance=3.0,  # Slightly larger tolerance for VIO
    )

    if success:
        print("[PASS] Formation flight completed")
    else:
        print("[FAIL] Formation flight failed")

    return success


def main():
    parser = argparse.ArgumentParser(description="Phase 6 GPS-Denied Navigation Test")
    parser.add_argument("--num-drones", type=int, default=3, help="Number of drones")
    parser.add_argument("--gps-denial-test", action="store_true", help="Run GPS denial test")
    parser.add_argument("--formation-test", action="store_true", help="Run formation test with VIO")
    parser.add_argument("--skip-simulation", action="store_true", help="Skip simulation startup (assume running)")
    args = parser.parse_args()

    print("=" * 60)
    print("PHASE 6: GPS-Denied Navigation Test")
    print("=" * 60)
    print(f"Number of drones: {args.num_drones}")
    print(f"VIO enabled: True")
    print(f"GPS denial test: {args.gps_denial_test}")
    print(f"Formation test: {args.formation_test}")

    # Create controller with VIO enabled
    config = SwarmConfig(
        num_drones=args.num_drones,
        enable_vio=True,
        vio_mode="hybrid",
    )
    controller = SwarmController(config)

    try:
        # Connect
        print("\nConnecting to drones...")
        if not controller.connect_all(timeout=30.0):
            print("[FAIL] Connection failed")
            return 1

        # Wait for EKF
        print("\nWaiting for EKF convergence...")
        if not controller.wait_for_ekf_all():
            print("[FAIL] EKF convergence failed")
            return 1

        # Arm and takeoff
        print("\nArming drones...")
        if not controller.set_mode_all("GUIDED"):
            print("[FAIL] Mode change failed")
            return 1

        if not controller.arm_all():
            print("[FAIL] Arming failed")
            return 1

        print("\nTaking off to 10m...")
        if not controller.takeoff_all(altitude=10.0, wait=True):
            print("[FAIL] Takeoff failed")
            return 1

        # Initialize VIO after takeoff (need GPS lock first)
        print("\nInitializing VIO...")
        test_vio_initialization(controller)

        # Run GPS denial test if requested
        if args.gps_denial_test:
            test_gps_denial(controller, drone_id=0)

        # Run formation test if requested
        if args.formation_test:
            test_formation_with_vio(controller)

        # Wait a bit to observe
        print("\nHolding position for 10 seconds...")
        time.sleep(10.0)

        # Land
        print("\nLanding...")
        controller.land_all()
        time.sleep(5.0)

        print("\n" + "=" * 60)
        print("TEST COMPLETED SUCCESSFULLY")
        print("=" * 60)
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 1
    except Exception as e:
        logger.exception(f"Test failed: {e}")
        return 1
    finally:
        controller.disconnect_all()


if __name__ == "__main__":
    sys.exit(main())
