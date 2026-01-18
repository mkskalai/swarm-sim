#!/usr/bin/env python3
"""All-in-one test runner for Phase 3: Swarm Coordination.

This script handles the complete test lifecycle:
1. Starts Gazebo with multi-drone world
2. Launches SITL instances with MAVProxy (UDP forwarding)
3. Runs the Phase 3 integration test
4. Cleans up all processes

Usage:
    python scripts/run_phase3_test.py                    # 3 drones, all tests
    python scripts/run_phase3_test.py --num-drones 6    # 6 drones
    python scripts/run_phase3_test.py --test formations # specific test
    python scripts/run_phase3_test.py --skip-gazebo     # if Gazebo already running
    python scripts/run_phase3_test.py --skip-test       # start sim only, no test

Prerequisites:
    - ArduPilot installed at ~/ardupilot
    - ardupilot_gazebo installed at ~/ardupilot_gazebo
    - Gazebo Harmonic installed
    - Python venv with pymavlink

Building ArduPilot:
    ArduPilot must be built BEFORE running this script. Build is required when:
    - First time setup (after cloning ArduPilot repository)
    - After git pull or switching branches
    - After modifying ArduPilot source code
    - After changing build configuration

    To build ArduPilot:
        cd ~/ardupilot
        ./waf configure --board sitl
        ./waf copter

    This script uses --no-rebuild to skip automatic rebuilding for faster startup.
    If you see "sim_vehicle.py: error: unrecognized arguments" or SITL fails to
    start, you may need to rebuild ArduPilot manually with the commands above.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


class ProcessManager:
    """Manages subprocesses for Gazebo and SITL instances."""

    def __init__(self):
        self.gazebo_proc: Optional[subprocess.Popen] = None
        self.sitl_procs: List[subprocess.Popen] = []
        self.log_dir = PROJECT_ROOT / "logs"
        self.log_dir.mkdir(exist_ok=True)
        self._shutting_down = False

    def start_gazebo(
        self,
        num_drones: int,
        world_path: Optional[Path] = None,
        verbose: int = 1,
    ) -> bool:
        """Start Gazebo with multi-drone world.

        Args:
            num_drones: Number of drones (determines world file)
            world_path: Optional custom world file path
            verbose: Gazebo verbosity level (0-4)

        Returns:
            True if Gazebo started successfully
        """
        if world_path is None:
            # Check if we have a generated world for this drone count
            generated_world = PROJECT_ROOT / "worlds" / f"multi_drone_{num_drones}.sdf"

            if generated_world.exists():
                world_path = generated_world
                print(f"Using generated world: {world_path}")
            elif num_drones <= 3:
                # Use the tutorial multi-drone world from ardupilot_gazebo (has 3 drones)
                # Try Docker location first
                ag_candidates = [
                    Path("/opt/ardupilot_gazebo"),
                    Path.home() / "ardupilot_gazebo",
                ]
                for ag_path in ag_candidates:
                    tutorial_world = ag_path / "worlds" / "tutorial_multi_drone.sdf"
                    if tutorial_world.exists():
                        world_path = tutorial_world
                        break
            else:
                # Need to generate a world for more drones
                print(f"Generating world for {num_drones} drones...")
                try:
                    from scripts.generate_world import generate_world
                    world_path = generate_world(num_drones)
                except Exception as e:
                    print(f"ERROR: Failed to generate world: {e}")
                    print("Try running: python scripts/generate_world.py --num-drones {num_drones}")
                    return False

        if not world_path.exists():
            print(f"ERROR: World file not found: {world_path}")
            print("Make sure ardupilot_gazebo is installed at ~/ardupilot_gazebo")
            return False

        print(f"Starting Gazebo with world: {world_path}")

        # Set up environment for Gazebo
        env = os.environ.copy()

        # Add ardupilot_gazebo paths - try Docker location first
        ag_candidates = [
            Path("/opt/ardupilot_gazebo"),  # Docker container location
            Path.home() / "ardupilot_gazebo",  # Local installation
        ]
        ag_path = None
        for candidate in ag_candidates:
            if candidate.exists():
                ag_path = candidate
                break
        if ag_path is None:
            ag_path = Path.home() / "ardupilot_gazebo"  # Fallback
        gz_plugin_path = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        gz_resource_path = env.get("GZ_SIM_RESOURCE_PATH", "")

        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{ag_path}/build:{gz_plugin_path}"
        env["GZ_SIM_RESOURCE_PATH"] = f"{ag_path}/models:{ag_path}/worlds:{gz_resource_path}"

        # Add project models and worlds directories
        env["GZ_SIM_RESOURCE_PATH"] = f"{PROJECT_ROOT}/models:{PROJECT_ROOT}/worlds:{env['GZ_SIM_RESOURCE_PATH']}"

        # Force NVIDIA GPU if available (for hybrid graphics)
        if Path("/usr/share/glvnd/egl_vendor.d/10_nvidia.json").exists():
            env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
            env["__EGL_VENDOR_LIBRARY_FILENAMES"] = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
            env["__NV_PRIME_RENDER_OFFLOAD"] = "1"

        # Clean LD_LIBRARY_PATH if running from VSCode snap
        if "snap/code" in env.get("LD_LIBRARY_PATH", ""):
            ld_paths = env.get("LD_LIBRARY_PATH", "").split(":")
            ld_paths = [p for p in ld_paths if "snap" not in p]
            env["LD_LIBRARY_PATH"] = ":".join(ld_paths)

        cmd = ["gz", "sim", f"-v{verbose}", "-r", str(world_path)]

        stdout_log = open(self.log_dir / "gazebo_stdout.log", "w")
        stderr_log = open(self.log_dir / "gazebo_stderr.log", "w")

        try:
            self.gazebo_proc = subprocess.Popen(
                cmd,
                stdout=stdout_log,
                stderr=stderr_log,
                env=env,
                preexec_fn=os.setsid,
            )
            print(f"Gazebo started (PID: {self.gazebo_proc.pid})")

            # Wait for Gazebo to initialize
            print("Waiting for Gazebo to initialize (10s)...")
            time.sleep(10)

            if self.gazebo_proc.poll() is not None:
                print("ERROR: Gazebo exited unexpectedly!")
                print(f"Check logs at: {self.log_dir}/gazebo_stderr.log")
                return False

            print("Gazebo ready!")
            return True

        except Exception as e:
            print(f"ERROR: Failed to start Gazebo: {e}")
            return False

    def start_sitl_instances(
        self,
        num_instances: int,
        base_port: int = 14540,
        startup_delay: float = 5.0,
    ) -> bool:
        """Start SITL instances with MAVProxy UDP forwarding.

        Uses sim_vehicle.py with --out flag to forward MAVLink to UDP ports.

        Args:
            num_instances: Number of SITL instances
            base_port: Base UDP port for MAVProxy forwarding
            startup_delay: Seconds to wait between instance launches

        Returns:
            True if all instances started successfully
        """
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
            print("ERROR: ArduCopter not found in any of the following locations:")
            for candidate in ardupilot_candidates:
                print(f"  - {candidate / 'ArduCopter'}")
            return False

        copter_path = ardupilot_path / "ArduCopter"
        print(f"Using ArduPilot at: {ardupilot_path}")

        print(f"\nLaunching {num_instances} SITL instances with MAVProxy...")

        for i in range(num_instances):
            udp_port = base_port + i

            # Build sim_vehicle.py command
            # -v ArduCopter: vehicle type
            # -f gazebo-iris: frame (with JSON model override)
            # --model JSON: use JSON interface for Gazebo
            # -I{i}: instance number
            # --out=udp:127.0.0.1:{port}: MAVProxy UDP forwarding
            # --console: show MAVProxy console window
            # --no-rebuild: skip rebuilding ArduPilot (must be pre-built)
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

            print(f"[SITL {i}] Starting on UDP port {udp_port}...")

            stdout_log = open(self.log_dir / f"sitl_{i}_stdout.log", "w")
            stderr_log = open(self.log_dir / f"sitl_{i}_stderr.log", "w")

            try:
                proc = subprocess.Popen(
                    cmd,
                    cwd=copter_path,
                    stdout=stdout_log,
                    stderr=stderr_log,
                    preexec_fn=os.setsid,
                )
                self.sitl_procs.append(proc)
                print(f"[SITL {i}] Started (PID: {proc.pid})")

            except Exception as e:
                print(f"[SITL {i}] Failed to start: {e}")
                return False

            # Wait between launches
            if i < num_instances - 1:
                print(f"Waiting {startup_delay}s before next instance...")
                time.sleep(startup_delay)

        # Wait for all SITL instances to be ready
        print("\nWaiting for all SITL instances to initialize (15s)...")
        time.sleep(15)

        # Check if any crashed
        for i, proc in enumerate(self.sitl_procs):
            if proc.poll() is not None:
                print(f"ERROR: SITL {i} exited unexpectedly!")
                print(f"Check logs at: {self.log_dir}/sitl_{i}_stderr.log")
                return False

        print(f"All {num_instances} SITL instances ready!")
        print("\nUDP ports for pymavlink connections:")
        for i in range(num_instances):
            print(f"  Drone {i}: udpin:0.0.0.0:{base_port + i}")

        return True

    def run_test(
        self,
        num_drones: int,
        test_type: str = "all",
        base_port: int = 14540,
    ) -> int:
        """Run the Phase 3 test script.

        Args:
            num_drones: Number of drones
            test_type: Test to run (all, formations, leader-follower, missions)
            base_port: Base UDP port

        Returns:
            Test exit code (0 = success)
        """
        test_script = PROJECT_ROOT / "scripts" / "test_phase3.py"

        cmd = [
            sys.executable,
            str(test_script),
            f"--num-drones={num_drones}",
            f"--base-port={base_port}",
            f"--test={test_type}",
        ]

        print(f"\n{'='*60}")
        print("RUNNING PHASE 3 TEST")
        print(f"{'='*60}\n")

        try:
            result = subprocess.run(cmd, cwd=PROJECT_ROOT)
            return result.returncode
        except KeyboardInterrupt:
            print("\nTest interrupted!")
            return 1

    def shutdown(self) -> None:
        """Shutdown all managed processes."""
        if self._shutting_down:
            return

        self._shutting_down = True
        print("\n" + "="*60)
        print("SHUTTING DOWN")
        print("="*60)

        # Stop SITL instances
        print("\nStopping SITL instances...")
        for i, proc in enumerate(self.sitl_procs):
            if proc.poll() is None:
                print(f"  [SITL {i}] Terminating...")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass

        # Wait for SITL processes
        for proc in self.sitl_procs:
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

        # Kill any remaining arducopter/mavproxy processes
        print("Cleaning up ArduCopter/MAVProxy processes...")
        for proc_name in ["arducopter", "mavproxy.py"]:
            try:
                subprocess.run(
                    ["pkill", "-f", proc_name],
                    capture_output=True,
                    timeout=3,
                )
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        # Stop Gazebo
        if self.gazebo_proc and self.gazebo_proc.poll() is None:
            print("Stopping Gazebo...")
            try:
                os.killpg(os.getpgid(self.gazebo_proc.pid), signal.SIGTERM)
                self.gazebo_proc.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                try:
                    os.killpg(os.getpgid(self.gazebo_proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

        # Kill any remaining gz processes
        try:
            subprocess.run(["pkill", "-f", "gz sim"], capture_output=True, timeout=3)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        print("Shutdown complete!")


def main():
    parser = argparse.ArgumentParser(
        description="All-in-one Phase 3 test runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/run_phase3_test.py                     # 3 drones, all tests
    python scripts/run_phase3_test.py --num-drones 6      # 6 drones
    python scripts/run_phase3_test.py --test formations   # formations only
    python scripts/run_phase3_test.py --skip-gazebo       # Gazebo already running
    python scripts/run_phase3_test.py --skip-test         # start sim, wait for manual testing

Note: Requires ardupilot_gazebo world with enough drones for your test.
The default tutorial_multi_drone.sdf has 3 drones. For 6 drones, you may
need to create a custom world or use generate_world.py.
        """,
    )
    parser.add_argument(
        "--num-drones", "-n",
        type=int,
        default=3,
        help="Number of drones (default: 3)",
    )
    parser.add_argument(
        "--test", "-t",
        type=str,
        default="all",
        choices=["all", "formations", "leader-follower", "missions"],
        help="Which test to run (default: all)",
    )
    parser.add_argument(
        "--base-port",
        type=int,
        default=14540,
        help="Base UDP port for MAVProxy (default: 14540)",
    )
    parser.add_argument(
        "--skip-gazebo",
        action="store_true",
        help="Skip Gazebo startup (use if already running)",
    )
    parser.add_argument(
        "--skip-sitl",
        action="store_true",
        help="Skip SITL startup (use if already running)",
    )
    parser.add_argument(
        "--world",
        type=str,
        help="Path to custom Gazebo world file",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=5.0,
        help="Seconds between SITL instance launches (default: 5.0)",
    )
    parser.add_argument(
        "--skip-test",
        action="store_true",
        help="Skip running test, just start Gazebo/SITL and wait (for manual testing)",
    )

    args = parser.parse_args()

    manager = ProcessManager()

    # Set up signal handlers
    def signal_handler(sig, frame):
        manager.shutdown()
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Start Gazebo
        if not args.skip_gazebo:
            world_path = Path(args.world) if args.world else None
            if not manager.start_gazebo(args.num_drones, world_path):
                print("\nFailed to start Gazebo!")
                manager.shutdown()
                return 1

        # Start SITL instances
        if not args.skip_sitl:
            if not manager.start_sitl_instances(
                args.num_drones,
                args.base_port,
                args.startup_delay,
            ):
                print("\nFailed to start SITL instances!")
                manager.shutdown()
                return 1

        # Run the test or wait
        if args.skip_test:
            print(f"\n{'='*60}")
            print("SIMULATION READY - Press Ctrl+C to shutdown")
            print(f"{'='*60}\n")
            print("Drones available at:")
            for i in range(args.num_drones):
                print(f"  Drone {i}: udpin:0.0.0.0:{args.base_port + i}")
            print("\nTo launch ROS2 bridge (in separate terminal):")
            print(f"  ros2 launch swarm_ros simulation.launch.py num_drones:={args.num_drones}")
            print()

            # Wait indefinitely until Ctrl+C
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            return 0
        else:
            exit_code = manager.run_test(
                args.num_drones,
                args.test,
                args.base_port,
            )
            return exit_code

    finally:
        manager.shutdown()


if __name__ == "__main__":
    sys.exit(main())
