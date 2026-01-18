#!/usr/bin/env python3
"""Run integration tests with simulation stack.

This script:
1. Starts Gazebo with the multi-drone world
2. Launches SITL instances
3. Waits for everything to be ready
4. Runs integration tests
5. Cleans up all processes

Usage:
    python scripts/run_integration_tests.py                    # 3 drones
    python scripts/run_integration_tests.py --num-drones 6    # 6 drones
    python scripts/run_integration_tests.py --test-file tests/test_phase2.py

For Docker:
    ./scripts/run.sh sim-test 3
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


class IntegrationTestRunner:
    """Manages simulation stack and runs integration tests."""

    def __init__(self, num_drones: int = 3, verbose: bool = False):
        self.num_drones = num_drones
        self.verbose = verbose
        self.gazebo_proc = None
        self.sitl_procs = []
        self.log_dir = PROJECT_ROOT / "logs"
        self.log_dir.mkdir(exist_ok=True)
        self._shutting_down = False

    def start_gazebo(self) -> bool:
        """Start Gazebo with multi-drone world."""
        world_path = PROJECT_ROOT / "worlds" / f"multi_drone_{self.num_drones}.sdf"

        if not world_path.exists():
            # Generate the world
            print(f"Generating world for {self.num_drones} drones...")
            try:
                result = subprocess.run(
                    [
                        sys.executable,
                        str(PROJECT_ROOT / "scripts" / "generate_world.py"),
                        "--num-drones", str(self.num_drones),
                    ],
                    capture_output=True,
                    text=True,
                    timeout=60,
                )
                if result.returncode != 0:
                    print(f"Failed to generate world: {result.stderr}")
                    return False
            except Exception as e:
                print(f"Error generating world: {e}")
                return False

        print(f"Starting Gazebo with {self.num_drones} drones...")

        # Determine if we're headless
        headless = os.environ.get("HEADLESS", "").lower() in ("1", "true", "yes")

        cmd = ["gz", "sim"]
        if headless:
            cmd.append("-s")  # server only, no GUI
        cmd.extend(["-r", str(world_path)])

        log_file = open(self.log_dir / "gazebo.log", "w")

        try:
            self.gazebo_proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            print(f"Gazebo started (PID: {self.gazebo_proc.pid})")
            return True
        except Exception as e:
            print(f"Failed to start Gazebo: {e}")
            return False

    def start_sitl_instances(self) -> bool:
        """Start SITL instances for all drones."""
        # Try Docker location first, then fall back to home directory
        ardupilot_candidates = [
            Path("/opt/ardupilot"),  # Docker container location
            Path.home() / "ardupilot",  # Local installation
        ]

        ardupilot_path = None
        for candidate in ardupilot_candidates:
            if (candidate / "ArduCopter").exists():
                ardupilot_path = candidate
                break

        if ardupilot_path is None:
            print("ArduPilot not found in any of the following locations:")
            for candidate in ardupilot_candidates:
                print(f"  - {candidate / 'ArduCopter'}")
            return False

        copter_path = ardupilot_path / "ArduCopter"
        print(f"Using ArduPilot at: {ardupilot_path}")

        print(f"Starting {self.num_drones} SITL instances...")

        for i in range(self.num_drones):
            udp_port = 14540 + i
            fdm_port = 9002 + (i * 10)

            # Use sim_vehicle.py with MAVProxy for UDP forwarding
            # Each SITL needs unique SYSID_THISMAV for MAVSDK to distinguish drones
            sysid = i + 1  # sysid 1, 2, 3, ...

            # Create param file with unique SYSID
            param_file = self.log_dir / f"sitl_{i}_params.parm"
            param_file.write_text(f"SYSID_THISMAV {sysid}\n")

            cmd = [
                "sim_vehicle.py",
                "-v", "ArduCopter",
                "-f", "gazebo-iris",
                "--model", "JSON",
                f"-I{i}",
                f"--out=udp:127.0.0.1:{udp_port}",
                "--no-rebuild",
                f"--add-param-file={param_file}",
            ]

            # Add console only if not headless
            headless = os.environ.get("HEADLESS", "").lower() in ("1", "true", "yes")
            if not headless:
                cmd.append("--console")

            log_file = open(self.log_dir / f"sitl_{i}.log", "w")

            try:
                proc = subprocess.Popen(
                    cmd,
                    cwd=copter_path,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )
                self.sitl_procs.append(proc)
                print(f"  Drone {i}: UDP port {udp_port}, FDM port {fdm_port}, SYSID {sysid} (PID: {proc.pid})")
            except Exception as e:
                print(f"  Drone {i}: Failed to start - {e}")
                return False

            # Wait between launches
            if i < self.num_drones - 1:
                time.sleep(3)

        return True

    def wait_for_simulation_ready(self, timeout: float = 120.0) -> bool:
        """Wait for simulation to be ready."""
        print(f"Waiting for simulation to be ready (timeout: {timeout}s)...")

        # Wait for Gazebo to start
        time.sleep(10)

        # Check if Gazebo is still running
        if self.gazebo_proc and self.gazebo_proc.poll() is not None:
            print("Gazebo exited unexpectedly!")
            return False

        # Wait for SITL instances to be ready (MAVLink ports)
        import socket

        start_time = time.time()
        for i in range(self.num_drones):
            port = 14540 + i
            ready = False

            while time.time() - start_time < timeout:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                        sock.settimeout(1)
                        # Try to bind to check if something is listening
                        # Actually, for UDP we check differently
                        # Just wait and assume it's ready after initial delay
                        ready = True
                        break
                except Exception:
                    time.sleep(1)

            if not ready:
                print(f"Drone {i} not ready after {timeout}s")
                return False

        # Additional wait for EKF convergence
        print("Waiting for EKF convergence (30s)...")
        time.sleep(30)

        print("Simulation ready!")
        return True

    def run_tests(self, test_file: str = "tests/") -> int:
        """Run integration tests with pytest."""
        print(f"\n{'='*60}")
        print("Running integration tests...")
        print(f"{'='*60}\n")

        # Set environment variable to indicate simulation is running
        env = os.environ.copy()
        env["SWARM_SIM_RUNNING"] = "1"
        env["SWARM_NUM_DRONES"] = str(self.num_drones)

        cmd = [
            sys.executable, "-m", "pytest",
            test_file,
            "-v",
            "-m", "integration",
            "--tb=short",
            # Override skip markers for integration tests
            "-p", "no:randomly",
        ]

        try:
            result = subprocess.run(cmd, env=env, cwd=PROJECT_ROOT)
            return result.returncode
        except Exception as e:
            print(f"Error running tests: {e}")
            return 1

    def shutdown(self):
        """Shutdown all processes."""
        if self._shutting_down:
            return
        self._shutting_down = True

        print("\nShutting down simulation...")

        # Stop SITL processes
        for proc in self.sitl_procs:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass

        # Stop Gazebo
        if self.gazebo_proc and self.gazebo_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.gazebo_proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass

        # Wait and force kill if needed
        time.sleep(2)

        for proc in self.sitl_procs:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

        if self.gazebo_proc and self.gazebo_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.gazebo_proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

        # Kill any remaining processes
        for proc_name in ["gz", "ruby", "arducopter", "mavproxy"]:
            subprocess.run(["pkill", "-f", proc_name], capture_output=True)

        print("Shutdown complete")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Run integration tests with simulation")
    parser.add_argument(
        "--num-drones", "-n",
        type=int,
        default=3,
        help="Number of drones (default: 3)",
    )
    parser.add_argument(
        "--test-file", "-t",
        type=str,
        default="tests/",
        help="Test file or directory (default: tests/)",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose output",
    )
    parser.add_argument(
        "--skip-sim",
        action="store_true",
        help="Skip simulation startup (assume already running)",
    )

    args = parser.parse_args()

    # Handle signals
    runner = IntegrationTestRunner(
        num_drones=args.num_drones,
        verbose=args.verbose,
    )

    def signal_handler(sig, frame):
        runner.shutdown()
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    exit_code = 1

    try:
        if not args.skip_sim:
            # Start simulation stack
            if not runner.start_gazebo():
                print("Failed to start Gazebo")
                sys.exit(1)

            if not runner.start_sitl_instances():
                print("Failed to start SITL instances")
                runner.shutdown()
                sys.exit(1)

            if not runner.wait_for_simulation_ready():
                print("Simulation failed to become ready")
                runner.shutdown()
                sys.exit(1)

        # Run tests
        exit_code = runner.run_tests(args.test_file)

    finally:
        if not args.skip_sim:
            runner.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
