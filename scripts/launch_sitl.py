#!/usr/bin/env python3
"""Launch multiple ArduPilot SITL instances for multi-drone simulation.

This script manages the lifecycle of multiple SITL instances, ensuring
proper port assignment and clean shutdown.

Usage:
    python scripts/launch_sitl.py --num-instances 3
    python scripts/launch_sitl.py -n 3 --headless --speedup 2.0
"""

import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from swarm.core.config import FleetConfig


@dataclass
class SITLProcess:
    """Represents a running SITL instance."""

    instance_id: int
    process: subprocess.Popen
    mavsdk_port: int
    fdm_port: int
    pid: int

    @property
    def is_running(self) -> bool:
        """Check if the process is still running."""
        return self.process.poll() is None


class SITLLauncher:
    """Manages multiple ArduPilot SITL instances.

    Example:
        with SITLLauncher(num_instances=3) as launcher:
            launcher.launch_all()
            # ... run tests ...
        # Processes are automatically cleaned up
    """

    def __init__(
        self,
        num_instances: int = 3,
        ardupilot_path: Optional[Path] = None,
        base_sitl_port: int = 5760,
        base_fdm_port: int = 9002,
    ):
        """Initialize SITL launcher.

        Args:
            num_instances: Number of SITL instances to manage.
            ardupilot_path: Path to ArduPilot repository. Defaults to ~/ardupilot.
            base_sitl_port: Base SITL TCP port for MAVSDK connections.
            base_fdm_port: Base port for FDM (JSON) interface.
        """
        self.num_instances = num_instances
        self.ardupilot_path = ardupilot_path or Path.home() / "ardupilot"
        self.config = FleetConfig(
            num_drones=num_instances,
            base_sitl_port=base_sitl_port,
            base_fdm_port=base_fdm_port,
        )
        self._processes: list[SITLProcess] = []
        self._shutting_down = False

    def _wait_for_port(self, port: int, timeout: float = 60.0, interval: float = 1.0) -> bool:
        """Wait for a TCP port to start listening.

        Args:
            port: TCP port number to check.
            timeout: Maximum seconds to wait.
            interval: Seconds between checks.

        Returns:
            True if port is listening within timeout.
        """
        start = time.time()
        while time.time() - start < timeout:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(1)
                    result = sock.connect_ex(("127.0.0.1", port))
                    if result == 0:
                        return True
            except (OSError, socket.error):
                pass
            time.sleep(interval)
        return False

    def launch_all(
        self,
        headless: bool = True,
        speedup: float = 1.0,
        startup_delay: float = 2.0,
        wait_ready: bool = True,
        ready_timeout: float = 60.0,
    ) -> bool:
        """Launch all SITL instances.

        Args:
            headless: Run without MAVProxy console (recommended for automation).
            speedup: Simulation speed multiplier.
            startup_delay: Seconds to wait between instance launches.
            wait_ready: Wait for each instance's TCP port to be listening.
            ready_timeout: Seconds to wait for each instance to be ready.

        Returns:
            True if all instances launched successfully.
        """
        copter_path = self.ardupilot_path / "ArduCopter"

        if not copter_path.exists():
            print(f"Error: ArduCopter not found at {copter_path}")
            return False

        print(f"Launching {self.num_instances} SITL instances...")
        print(f"ArduPilot path: {self.ardupilot_path}")
        print(f"Headless: {headless}, Speedup: {speedup}x")
        print()

        for i in range(self.num_instances):
            mavsdk_port = self.config.get_mavsdk_port(i)
            fdm_port = self.config.get_fdm_port(i)

            # Build command - run arducopter directly with JSON model
            # Note: -I sets instance number, which determines:
            #   - FDM port for Gazebo: 9002 + instance*10
            #   - Native SITL MAVLink TCP port: 5760 + instance*10
            arducopter_binary = self.ardupilot_path / "build" / "sitl" / "bin" / "arducopter"
            autotest_dir = self.ardupilot_path / "Tools" / "autotest"

            cmd = [
                str(arducopter_binary),
                "-S",
                "--model", "JSON",
                "--speedup", str(int(speedup)),
                "--defaults", f"{autotest_dir}/default_params/copter.parm,{autotest_dir}/default_params/gazebo-iris.parm",
                "--sim-address", "127.0.0.1",
                f"-I{i}",
            ]

            sitl_port = 5760 + (i * 10)  # SITL native TCP port
            print(f"[Instance {i}] SITL TCP: {sitl_port}, FDM port: {fdm_port}")

            # Launch process
            # Note: sim_vehicle.py spawns xterm windows for each SITL instance.
            # Use log files to capture output for debugging.
            log_dir = PROJECT_ROOT / "logs"
            log_dir.mkdir(exist_ok=True)
            stdout_log = open(log_dir / f"sitl_{i}_stdout.log", "w")
            stderr_log = open(log_dir / f"sitl_{i}_stderr.log", "w")

            try:
                proc = subprocess.Popen(
                    cmd,
                    cwd=copter_path,
                    stdout=stdout_log,
                    stderr=stderr_log,
                    # Create new process group for clean shutdown
                    preexec_fn=os.setsid,
                )

                self._processes.append(SITLProcess(
                    instance_id=i,
                    process=proc,
                    mavsdk_port=mavsdk_port,
                    fdm_port=fdm_port,
                    pid=proc.pid,
                ))

                print(f"[Instance {i}] Started (PID: {proc.pid})")

            except Exception as e:
                print(f"[Instance {i}] Failed to start: {e}")
                self.shutdown_all()
                return False

            # Wait for SITL to be ready (TCP port listening)
            if wait_ready:
                print(f"[Instance {i}] Waiting for TCP port {sitl_port}...", end=" ", flush=True)
                if self._wait_for_port(sitl_port, timeout=ready_timeout):
                    print("Ready!")
                else:
                    print("TIMEOUT!")
                    print(f"[Instance {i}] Failed to start within {ready_timeout}s")
                    self.shutdown_all()
                    return False
            # Delay between launches to reduce port conflicts
            elif i < self.num_instances - 1:
                time.sleep(startup_delay)

        print(f"\nAll {self.num_instances} instances launched!")
        print("\nTo connect MAVSDK (headless mode uses TCP):")
        for sitl in self._processes:
            sitl_port = 5760 + (sitl.instance_id * 10)
            print(f"  Drone {sitl.instance_id}: tcpout://127.0.0.1:{sitl_port}")

        return True

    def shutdown_all(self) -> None:
        """Shutdown all SITL instances gracefully."""
        if self._shutting_down:
            return

        self._shutting_down = True
        print("\nShutting down SITL instances...")

        # Try to kill sim_vehicle.py process groups (if still running)
        for sitl in self._processes:
            if sitl.is_running:
                print(f"[Instance {sitl.instance_id}] Terminating (PID: {sitl.pid})...")
                try:
                    os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass

        # Wait for sim_vehicle.py processes
        for sitl in self._processes:
            try:
                sitl.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(sitl.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

        # Kill actual SITL processes (arducopter binaries spawned by sim_vehicle.py)
        print("Stopping ArduCopter processes...")
        try:
            subprocess.run(
                ["pkill", "-f", "arducopter"],
                capture_output=True,
                timeout=5,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        self._processes.clear()
        print("All instances shut down")

    def get_status(self) -> dict:
        """Get status of all SITL instances.

        Returns:
            Dictionary with instance status information.
        """
        return {
            "total": self.num_instances,
            "running": sum(1 for p in self._processes if p.is_running),
            "instances": [
                {
                    "id": p.instance_id,
                    "pid": p.pid,
                    "running": p.is_running,
                    "mavsdk_port": p.mavsdk_port,
                    "fdm_port": p.fdm_port,
                }
                for p in self._processes
            ],
        }

    def wait(self) -> None:
        """Wait until interrupted by Ctrl+C.

        Note: sim_vehicle.py spawns child processes (actual SITL) and exits.
        We wait indefinitely rather than tracking process state.
        """
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown_all()


def main():
    parser = argparse.ArgumentParser(
        description="Launch multiple ArduPilot SITL instances",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/launch_sitl.py --num-instances 3
    python scripts/launch_sitl.py -n 3 --headless --speedup 2.0

Before running, ensure Gazebo is started:
    cd ~/ardupilot_gazebo
    gz sim -r ~/swarm/worlds/multi_drone_3.sdf

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
        "--headless",
        action="store_true",
        default=True,
        help="Run without MAVProxy console (default: True)",
    )
    parser.add_argument(
        "--no-headless",
        action="store_false",
        dest="headless",
        help="Run with MAVProxy console",
    )
    parser.add_argument(
        "--speedup",
        type=float,
        default=1.0,
        help="Simulation speed multiplier (default: 1.0)",
    )
    parser.add_argument(
        "--ardupilot-path",
        type=str,
        help="Path to ArduPilot repository (default: ~/ardupilot)",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=2.0,
        help="Seconds between instance launches (default: 2.0)",
    )

    args = parser.parse_args()

    ardupilot_path = Path(args.ardupilot_path) if args.ardupilot_path else None

    # Handle Ctrl+C gracefully
    launcher = SITLLauncher(
        num_instances=args.num_instances,
        ardupilot_path=ardupilot_path,
    )

    def signal_handler(sig, frame):
        print("\nReceived interrupt signal...")
        launcher.shutdown_all()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        success = launcher.launch_all(
            headless=args.headless,
            speedup=args.speedup,
            startup_delay=args.startup_delay,
        )

        if success:
            print("\nSITL instances running. Press Ctrl+C to stop.")
            launcher.wait()
        else:
            sys.exit(1)

    finally:
        launcher.shutdown_all()


if __name__ == "__main__":
    main()
