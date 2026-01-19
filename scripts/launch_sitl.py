#!/usr/bin/env python3
"""Launch multiple ArduPilot SITL instances for multi-drone simulation.

This script is a CLI wrapper around the swarm.simulation.sitl_launcher module.

Usage:
    python scripts/launch_sitl.py --num-instances 3
    python scripts/launch_sitl.py -n 6 --startup-delay 8
"""

import argparse
import signal
import sys
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from swarm.simulation import SITLLauncher


def main():
    parser = argparse.ArgumentParser(
        description="Launch multiple ArduPilot SITL instances with MAVProxy UDP forwarding",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/launch_sitl.py --num-instances 3
    python scripts/launch_sitl.py -n 6 --startup-delay 8

Before running, ensure Gazebo is started:
    source scripts/setup_env.sh
    gz sim -r worlds/generated_3.sdf

This will spawn MAVProxy console windows (xterm) for each SITL instance,
forwarding MAVLink to UDP ports 14540, 14541, 14542, etc.

Press Ctrl+C to shut down all instances.
        """,
    )
    parser.add_argument(
        "-n", "--num-instances",
        type=int,
        default=3,
        help="Number of SITL instances (default: 3)",
    )
    parser.add_argument(
        "--ardupilot-path",
        type=str,
        help="Path to ArduPilot repository (default: auto-detect)",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=5.0,
        help="Seconds between instance launches (default: 5.0)",
    )
    parser.add_argument(
        "--base-port",
        type=int,
        default=14540,
        help="Base UDP port for MAVProxy forwarding (default: 14540)",
    )

    args = parser.parse_args()

    ardupilot_path = Path(args.ardupilot_path) if args.ardupilot_path else None

    launcher = SITLLauncher(
        num_instances=args.num_instances,
        ardupilot_path=ardupilot_path,
        base_udp_port=args.base_port,
    )

    def signal_handler(sig, frame):
        print("\nReceived interrupt signal...")
        launcher.shutdown_all()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        success = launcher.launch_all(startup_delay=args.startup_delay)

        if success:
            print("\nSITL instances running. Press Ctrl+C to stop.")
            try:
                while True:
                    import time
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
        else:
            sys.exit(1)

    finally:
        launcher.shutdown_all()


if __name__ == "__main__":
    main()
