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
        base_udp_port: int = 14540,
        base_fdm_port: int = 9002,
    ):
        """Initialize SITL launcher.

        Args:
            num_instances: Number of SITL instances to manage.
            ardupilot_path: Path to ArduPilot repository. Defaults to ~/ardupilot.
            base_udp_port: Base UDP port for MAVProxy forwarding (pymavlink).
            base_fdm_port: Base port for FDM (JSON) interface.
        """
        self.num_instances = num_instances
        self.ardupilot_path = ardupilot_path or Path.home() / "ardupilot"
        self.base_udp_port = base_udp_port
        self.config = FleetConfig(
            num_drones=num_instances,
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
        headless: bool = False,
        speedup: float = 1.0,
        startup_delay: float = 5.0,
        wait_ready: bool = True,
        ready_timeout: float = 60.0,
    ) -> bool:
        """Launch all SITL instances with MAVProxy UDP forwarding.

        Args:
            headless: Run without MAVProxy console (not recommended - breaks UDP).
            speedup: Simulation speed multiplier.
            startup_delay: Seconds to wait between instance launches.
            wait_ready: Wait for each instance's UDP port to be available.
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
        print(f"Speedup: {speedup}x")
        print()

        # Create log directory
        log_dir = PROJECT_ROOT / "logs"
        log_dir.mkdir(exist_ok=True)

        for i in range(self.num_instances):
            udp_port = self.base_udp_port + i  # 14540, 14541, 14542, ...
            fdm_port = self.config.get_fdm_port(i)

            # Use sim_vehicle.py with MAVProxy for UDP forwarding
            # This spawns xterm consoles and forwards MAVLink to UDP ports
            cmd = [
                "sim_vehicle.py",
                "-v", "ArduCopter",
                "-f", "gazebo-iris",
                "--model", "JSON",
                f"-I{i}",
                f"--out=udp:127.0.0.1:{udp_port}",
                "--console",
                "--no-rebuild",
            ]

            print(f"[Instance {i}] UDP port: {udp_port}, FDM port: {fdm_port}")

            # Log files for debugging
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
                    mavsdk_port=udp_port,
                    fdm_port=fdm_port,
                    pid=proc.pid,
                ))

                print(f"[Instance {i}] Started (PID: {proc.pid})")

            except Exception as e:
                print(f"[Instance {i}] Failed to start: {e}")
                self.shutdown_all()
                return False

            # Wait between launches - sim_vehicle.py needs time to start
            if i < self.num_instances - 1:
                print(f"[Instance {i}] Waiting {startup_delay}s before next instance...")
                time.sleep(startup_delay)

        # Wait for all instances to be ready
        if wait_ready:
            print("\nWaiting for all instances to be ready...")
            time.sleep(10)  # Give MAVProxy time to initialize

        print(f"\nAll {self.num_instances} instances launched!")
        print("\nUDP ports for SwarmBridge/pymavlink:")
        for sitl in self._processes:
            print(f"  Drone {sitl.instance_id}: udpin://0.0.0.0:{sitl.mavsdk_port}")

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

        # Kill all related processes spawned by sim_vehicle.py
        print("Stopping ArduCopter/MAVProxy processes...")
        for proc_name in ["arducopter", "mavproxy.py", "MAVProxy"]:
            try:
                subprocess.run(
                    ["pkill", "-f", proc_name],
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
        description="Launch multiple ArduPilot SITL instances with MAVProxy UDP forwarding",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/launch_sitl.py --num-instances 3
    python scripts/launch_sitl.py -n 6 --startup-delay 8

Before running, ensure Gazebo is started:
    source scripts/setup_env.sh
    gz sim -r worlds/perception_test.sdf

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
        default=5.0,
        help="Seconds between instance launches (default: 5.0)",
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
