"""SITL launcher for managing ArduPilot SITL instances.

This module provides utilities to launch and manage multiple ArduPilot SITL
instances for multi-drone simulation.
"""

import os
import signal
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .world_generator import get_fdm_port


# Use SWARM_PROJECT_ROOT env var if set (avoids paths with spaces)
PROJECT_ROOT = Path(os.environ.get("SWARM_PROJECT_ROOT", Path(__file__).parent.parent.parent))


@dataclass
class SITLProcess:
    """Represents a running SITL instance."""

    instance_id: int
    process: subprocess.Popen
    mavlink_port: int
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
        speedup: float = 1.0,
    ):
        """Initialize SITL launcher.

        Args:
            num_instances: Number of SITL instances to manage.
            ardupilot_path: Path to ArduPilot repository. Auto-detected if None.
            base_udp_port: Base UDP port for MAVProxy forwarding (pymavlink).
            base_fdm_port: Base port for FDM (JSON) interface.
            speedup: Simulation speed multiplier (1.0 = real-time, 2.0 = 2x speed).
        """
        self.num_instances = num_instances
        self.ardupilot_path = self._find_ardupilot(ardupilot_path)
        self.base_udp_port = base_udp_port
        self.base_fdm_port = base_fdm_port
        self.speedup = speedup
        self._processes: list[SITLProcess] = []
        self._shutting_down = False
        self._log_dir = PROJECT_ROOT / "logs"

    def _find_ardupilot(self, path: Optional[Path]) -> Path:
        """Find ArduPilot installation path."""
        if path is not None:
            return path

        candidates = [
            Path("/opt/ardupilot"),  # Docker container location
            Path.home() / "ardupilot",  # Local installation
        ]

        for candidate in candidates:
            if (candidate / "ArduCopter").exists():
                return candidate

        # Return default for error message
        return Path.home() / "ardupilot"

    def _wait_for_port(
        self,
        port: int,
        timeout: float = 60.0,
        interval: float = 1.0,
    ) -> bool:
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
        startup_delay: float = 5.0,
        wait_ready: bool = True,
        ready_timeout: float = 60.0,
    ) -> bool:
        """Launch all SITL instances with MAVProxy UDP forwarding.

        Args:
            startup_delay: Seconds to wait between instance launches.
            wait_ready: Wait for instances to be ready.
            ready_timeout: Seconds to wait for readiness.

        Returns:
            True if all instances launched successfully.
        """
        copter_path = self.ardupilot_path / "ArduCopter"

        if not copter_path.exists():
            print(f"Error: ArduCopter not found at {copter_path}")
            return False

        print(f"Launching {self.num_instances} SITL instances...")
        print(f"ArduPilot path: {self.ardupilot_path}")
        if self.speedup != 1.0:
            print(f"Speedup: {int(self.speedup)}x")
        print()

        # Create log directory
        self._log_dir.mkdir(exist_ok=True)

        for i in range(self.num_instances):
            udp_port = self.base_udp_port + i
            fdm_port = get_fdm_port(i, self.base_fdm_port)
            sysid = i + 1  # SYSID: 1, 2, 3, ...

            # Create param file with unique SYSID
            param_file = self._log_dir / f"sitl_{i}_params.parm"
            param_file.write_text(f"SYSID_THISMAV {sysid}\n")

            cmd = [
                "sim_vehicle.py",
                "-v", "ArduCopter",
                "-f", "gazebo-iris",
                "--model", "JSON",
                f"-I{i}",
                f"--out=udp:127.0.0.1:{udp_port}",
                "--console",
                "--no-rebuild",
                f"--add-param-file={param_file}",
                f"--speedup={int(self.speedup)}",
            ]

            print(f"[SITL {i}] UDP port: {udp_port}, FDM port: {fdm_port}, SYSID: {sysid}")

            stdout_log = open(self._log_dir / f"sitl_{i}_stdout.log", "w")
            stderr_log = open(self._log_dir / f"sitl_{i}_stderr.log", "w")

            try:
                proc = subprocess.Popen(
                    cmd,
                    cwd=copter_path,
                    stdout=stdout_log,
                    stderr=stderr_log,
                    preexec_fn=os.setsid,
                )

                self._processes.append(SITLProcess(
                    instance_id=i,
                    process=proc,
                    mavlink_port=udp_port,
                    fdm_port=fdm_port,
                    pid=proc.pid,
                ))

                print(f"[SITL {i}] Started (PID: {proc.pid})")

            except Exception as e:
                print(f"[SITL {i}] Failed to start: {e}")
                self.shutdown_all()
                return False

            # Wait between launches
            if i < self.num_instances - 1:
                print(f"Waiting {startup_delay}s before next instance...")
                time.sleep(startup_delay)

        # Wait for all instances to be ready
        if wait_ready:
            print("\nWaiting for all instances to initialize...")
            time.sleep(15)

            # Check if any crashed
            for sitl in self._processes:
                if not sitl.is_running:
                    print(f"ERROR: SITL {sitl.instance_id} exited unexpectedly!")
                    print(f"Check logs at: {self._log_dir}/sitl_{sitl.instance_id}_stderr.log")
                    return False

        print(f"\nAll {self.num_instances} SITL instances launched!")
        print("\nUDP ports for pymavlink connections:")
        for sitl in self._processes:
            print(f"  Drone {sitl.instance_id}: udpin:0.0.0.0:{sitl.mavlink_port}")

        return True

    def shutdown_all(self) -> None:
        """Shutdown all SITL instances gracefully."""
        if self._shutting_down:
            return

        self._shutting_down = True
        print("\nShutting down SITL instances...")

        # Try to kill sim_vehicle.py process groups
        for sitl in self._processes:
            if sitl.is_running:
                print(f"[SITL {sitl.instance_id}] Terminating (PID: {sitl.pid})...")
                try:
                    os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass

        # Wait for processes
        for sitl in self._processes:
            try:
                sitl.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(sitl.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

        # Kill all related processes
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
        self._shutting_down = False
        print("All SITL instances shut down")

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
                    "mavlink_port": p.mavlink_port,
                    "fdm_port": p.fdm_port,
                }
                for p in self._processes
            ],
        }

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown_all()
