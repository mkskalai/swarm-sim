"""Simulation manager for orchestrating ROS2 + Gazebo + SITL.

This module provides the SimManager class that handles the complete simulation
stack including Gazebo, SITL instances, and optional ROS2 nodes.
"""

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional

from .world_generator import WorldGenerator
from .sitl_launcher import SITLLauncher


# Use SWARM_PROJECT_ROOT env var if set (avoids paths with spaces)
# Falls back to file-based resolution
PROJECT_ROOT = Path(os.environ.get("SWARM_PROJECT_ROOT", Path(__file__).parent.parent.parent))


class SimManager:
    """Manages ROS2 + Gazebo + SITL processes for simulation.

    Default mode starts the full stack including ROS2 nodes.
    Use `use_ros=False` for algorithm testing without ROS2.

    Usage:
        # Full stack (default)
        sim = SimManager(num_drones=6, world="complex")

        if sim.start():
            # Run tests via SwarmBridge (ROS2)
            sim.stop()

        # Or as context manager:
        with SimManager(num_drones=6) as sim:
            # Tests run here...

        # Without ROS2 (algorithm testing only):
        sim = SimManager(num_drones=3, use_ros=False)
    """

    def __init__(
        self,
        num_drones: int = 3,
        world: str = "complex",
        layout: str = "grid",
        spacing: float = 5.0,
        headless: bool = False,
        use_ros: bool = True,
        ardupilot_path: Optional[Path] = None,
        base_udp_port: int = 14540,
        base_fdm_port: int = 9002,
        real_time_factor: float = 1.0,
    ):
        """Initialize simulation manager.

        Args:
            num_drones: Number of drones to spawn
            world: World type ("basic" or "complex")
            layout: Spawn layout ("grid" or "line")
            spacing: Distance between drones in meters
            headless: Run without Gazebo GUI
            use_ros: Start ROS2 nodes (default True)
            ardupilot_path: Path to ArduPilot (auto-detected if None)
            base_udp_port: Base UDP port for MAVProxy
            base_fdm_port: Base FDM port for Gazebo-SITL
            real_time_factor: Simulation speed (1.0 = real-time, 2.0 = 2x)
        """
        self.num_drones = num_drones
        self.world = world
        self.layout = layout
        self.spacing = spacing
        self.headless = headless
        self.use_ros = use_ros
        self.base_udp_port = base_udp_port
        self.base_fdm_port = base_fdm_port
        self.real_time_factor = real_time_factor

        self._ardupilot_path = self._find_ardupilot(ardupilot_path)
        self._gazebo_proc: Optional[subprocess.Popen] = None
        self._sitl_launcher: Optional[SITLLauncher] = None
        self._ros_procs: list[subprocess.Popen] = []
        self._log_dir = PROJECT_ROOT / "logs"
        self._shutting_down = False

    def _find_ardupilot(self, path: Optional[Path]) -> Path:
        """Find ArduPilot installation path."""
        if path is not None:
            return path

        candidates = [
            Path("/opt/ardupilot"),
            Path.home() / "ardupilot",
        ]

        for candidate in candidates:
            if (candidate / "ArduCopter").exists():
                return candidate

        return Path.home() / "ardupilot"

    def _find_ardupilot_gazebo(self) -> Path:
        """Find ardupilot_gazebo installation path."""
        candidates = [
            Path("/opt/ardupilot_gazebo"),
            Path.home() / "ardupilot_gazebo",
        ]

        for candidate in candidates:
            if candidate.exists():
                return candidate

        return Path.home() / "ardupilot_gazebo"

    def start(self, timeout: float = 120.0) -> bool:
        """Start full simulation stack.

        Order:
        1. Generate world with drone models (grid/line layout)
        2. Start Gazebo with generated world
        3. Start SITL instances (one per drone)
        4. Start ROS2 nodes (if use_ros=True)
        5. Wait for EKF convergence

        Args:
            timeout: Maximum time to wait for startup

        Returns:
            True if all processes started and ready.
        """
        self._log_dir.mkdir(exist_ok=True)

        # 1. Generate world
        print("\n" + "=" * 60)
        print("PREPARING WORLD")
        print("=" * 60)
        try:
            world_path = WorldGenerator.generate(
                self.num_drones,
                self.world,
                self.layout,
                self.spacing,
                self.real_time_factor,
            )
        except Exception as e:
            print(f"ERROR: Failed to generate world: {e}")
            return False

        # 2. Start Gazebo
        if not self._start_gazebo(world_path):
            return False

        # 3. Start SITL instances
        if not self._start_sitl_instances():
            self.stop()
            return False

        # 4. Start ROS2 nodes
        if self.use_ros:
            if not self._start_ros_nodes():
                self.stop()
                return False

        print("\n" + "=" * 60)
        print("SIMULATION READY")
        print("=" * 60)
        print(f"\nDrones available at:")
        for i in range(self.num_drones):
            print(f"  Drone {i}: udpin:0.0.0.0:{self.base_udp_port + i}")

        return True

    def _start_gazebo(self, world_path: Path, verbose: int = 1) -> bool:
        """Start Gazebo with the specified world.

        Args:
            world_path: Path to world SDF file
            verbose: Gazebo verbosity level (0-4)

        Returns:
            True if Gazebo started successfully
        """
        if not world_path.exists():
            print(f"ERROR: World file not found: {world_path}")
            return False

        print("\n" + "=" * 60)
        print("STARTING GAZEBO")
        print("=" * 60)
        print(f"Loading world: {world_path}")

        # Set up environment
        env = os.environ.copy()

        # Add ardupilot_gazebo paths
        ag_path = self._find_ardupilot_gazebo()
        gz_plugin_path = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        gz_resource_path = env.get("GZ_SIM_RESOURCE_PATH", "")

        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{ag_path}/build:{gz_plugin_path}"
        env["GZ_SIM_RESOURCE_PATH"] = f"{ag_path}/models:{ag_path}/worlds:{gz_resource_path}"

        # Add project models and worlds
        env["GZ_SIM_RESOURCE_PATH"] = f"{PROJECT_ROOT}/models:{PROJECT_ROOT}/worlds:{env['GZ_SIM_RESOURCE_PATH']}"

        # Force NVIDIA GPU if available
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
        if self.headless:
            cmd.insert(2, "-s")  # Server mode (no GUI)

        stdout_log = open(self._log_dir / "gazebo_stdout.log", "w")
        stderr_log = open(self._log_dir / "gazebo_stderr.log", "w")

        try:
            self._gazebo_proc = subprocess.Popen(
                cmd,
                stdout=stdout_log,
                stderr=stderr_log,
                env=env,
                preexec_fn=os.setsid,
            )
            print(f"Gazebo started (PID: {self._gazebo_proc.pid})")

            # Wait for Gazebo to initialize
            print("Waiting for Gazebo to initialize (10s)...")
            time.sleep(10)

            if self._gazebo_proc.poll() is not None:
                print("ERROR: Gazebo exited unexpectedly!")
                print(f"Check logs at: {self._log_dir}/gazebo_stderr.log")
                return False

            print("Gazebo ready!")
            return True

        except Exception as e:
            print(f"ERROR: Failed to start Gazebo: {e}")
            return False

    def _start_sitl_instances(self) -> bool:
        """Start all SITL instances.

        Returns:
            True if all instances started successfully
        """
        print("\n" + "=" * 60)
        print("STARTING SITL INSTANCES")
        print("=" * 60)

        self._sitl_launcher = SITLLauncher(
            num_instances=self.num_drones,
            ardupilot_path=self._ardupilot_path,
            base_udp_port=self.base_udp_port,
            base_fdm_port=self.base_fdm_port,
            speedup=self.real_time_factor,
        )

        return self._sitl_launcher.launch_all()

    def _start_ros_nodes(self) -> bool:
        """Start ROS2 launch file with SwarmBridge and other nodes.

        Returns:
            True if ROS2 nodes started successfully
        """
        print("\n" + "=" * 60)
        print("STARTING ROS2 NODES")
        print("=" * 60)

        cmd = [
            "ros2", "launch", "swarm_ros", "simulation.launch.py",
            f"num_drones:={self.num_drones}",
        ]

        stdout_log = open(self._log_dir / "ros2_stdout.log", "w")
        stderr_log = open(self._log_dir / "ros2_stderr.log", "w")

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=stdout_log,
                stderr=stderr_log,
                preexec_fn=os.setsid,
            )
            self._ros_procs.append(proc)
            print(f"ROS2 launch started (PID: {proc.pid})")

            # Wait for ROS2 to initialize
            time.sleep(5)

            if proc.poll() is not None:
                print("ERROR: ROS2 launch exited unexpectedly!")
                print(f"Check logs at: {self._log_dir}/ros2_stderr.log")
                return False

            return True

        except FileNotFoundError:
            print("WARNING: ros2 command not found, skipping ROS2 nodes")
            return True  # Not a fatal error
        except Exception as e:
            print(f"ERROR: Failed to start ROS2 nodes: {e}")
            return False

    def stop(self) -> None:
        """Stop all simulation processes (reverse order)."""
        if self._shutting_down:
            return

        self._shutting_down = True
        print("\n" + "=" * 60)
        print("SHUTTING DOWN SIMULATION")
        print("=" * 60)

        # Stop ROS2 nodes
        if self._ros_procs:
            print("\nStopping ROS2 nodes...")
            for proc in self._ros_procs:
                if proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        proc.wait(timeout=5)
                    except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                        try:
                            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                        except (ProcessLookupError, PermissionError):
                            pass

        # Stop SITL instances
        if self._sitl_launcher:
            self._sitl_launcher.shutdown_all()

        # Stop Gazebo
        if self._gazebo_proc and self._gazebo_proc.poll() is None:
            print("\nStopping Gazebo...")
            try:
                os.killpg(os.getpgid(self._gazebo_proc.pid), signal.SIGTERM)
                self._gazebo_proc.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                try:
                    os.killpg(os.getpgid(self._gazebo_proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

        # Kill any remaining gz processes
        try:
            subprocess.run(["pkill", "-f", "gz sim"], capture_output=True, timeout=3)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        self._ros_procs.clear()
        self._gazebo_proc = None
        self._sitl_launcher = None
        self._shutting_down = False

        print("\nShutdown complete!")

    def wait(self) -> None:
        """Wait until interrupted by Ctrl+C."""
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    def __enter__(self):
        if not self.start():
            raise RuntimeError("Failed to start simulation")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
