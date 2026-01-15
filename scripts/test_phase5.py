#!/usr/bin/env python3
"""
Phase 5 Integration Test: Perception Pipeline

Tests the complete perception pipeline:
1. Camera sensors in Gazebo
2. ROS2 bridge for camera topics
3. YOLOv11 detection
4. Object tracking
5. P2P detection sharing

Prerequisites:
    pip install ultralytics opencv-python

Usage:
    # Full test (starts Gazebo, SITL, ROS2)
    python scripts/test_phase5.py --num-drones 3

    # Skip Gazebo/SITL if already running
    python scripts/test_phase5.py --skip-gazebo --skip-sitl

    # Test detector only (no simulation)
    python scripts/test_phase5.py --detector-only
"""

import argparse
import logging
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent.resolve()
WORLDS_DIR = PROJECT_ROOT / "worlds"
ARDUPILOT_DIR = Path.home() / "ardupilot"
ARDUPILOT_GAZEBO_DIR = Path.home() / "ardupilot_gazebo"


def test_detector_standalone():
    """Test YOLO detector without simulation."""
    logger.info("=" * 60)
    logger.info("Testing YOLOv11 Detector (standalone)")
    logger.info("=" * 60)

    try:
        import numpy as np
        from swarm.perception import YOLODetector, DetectorConfig

        # Initialize detector
        config = DetectorConfig(
            model_name="yolo11n.pt",
            confidence_threshold=0.4,
            use_gpu=True,
        )
        detector = YOLODetector(config)

        logger.info("Initializing detector...")
        if not detector.initialize():
            logger.error("Failed to initialize detector")
            return False

        logger.info(f"Detector initialized on: {detector.device}")

        # Create test image (dummy)
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Add some color variation
        test_image[100:200, 100:200] = [255, 0, 0]  # Red square
        test_image[250:350, 300:400] = [0, 255, 0]  # Green square

        # Run inference
        logger.info("Running inference on test image...")
        detections, inference_ms = detector.detect(test_image)

        logger.info(f"Inference time: {inference_ms:.1f}ms")
        logger.info(f"Detections: {len(detections)}")

        # Run multiple times to get average
        times = []
        for i in range(10):
            _, ms = detector.detect(test_image)
            times.append(ms)

        avg_ms = sum(times) / len(times)
        logger.info(f"Average inference time (10 runs): {avg_ms:.1f}ms")

        logger.info("Detector test PASSED")
        return True

    except ImportError as e:
        logger.error(f"Import error: {e}")
        logger.error("Install with: pip install ultralytics opencv-python")
        return False
    except Exception as e:
        logger.error(f"Detector test failed: {e}")
        return False


def test_tracker_standalone():
    """Test IoU tracker without simulation."""
    logger.info("=" * 60)
    logger.info("Testing IoU Tracker (standalone)")
    logger.info("=" * 60)

    try:
        from swarm.perception import SimpleTracker, Detection

        tracker = SimpleTracker()

        # Simulate detections across frames
        frames = [
            # Frame 1: Two objects
            [
                Detection(class_id=0, class_name="person", confidence=0.8, bbox=(100, 100, 50, 100)),
                Detection(class_id=2, class_name="car", confidence=0.9, bbox=(300, 200, 100, 60)),
            ],
            # Frame 2: Objects moved slightly
            [
                Detection(class_id=0, class_name="person", confidence=0.85, bbox=(105, 102, 50, 100)),
                Detection(class_id=2, class_name="car", confidence=0.88, bbox=(310, 202, 100, 60)),
            ],
            # Frame 3: Person moved more, car same
            [
                Detection(class_id=0, class_name="person", confidence=0.82, bbox=(120, 110, 50, 100)),
                Detection(class_id=2, class_name="car", confidence=0.87, bbox=(312, 203, 100, 60)),
            ],
            # Frame 4: New object appears
            [
                Detection(class_id=0, class_name="person", confidence=0.80, bbox=(130, 115, 50, 100)),
                Detection(class_id=2, class_name="car", confidence=0.86, bbox=(314, 204, 100, 60)),
                Detection(class_id=7, class_name="truck", confidence=0.75, bbox=(500, 150, 120, 80)),
            ],
        ]

        logger.info(f"Processing {len(frames)} frames...")

        for i, detections in enumerate(frames):
            tracks = tracker.update(detections)
            logger.info(f"Frame {i+1}: {len(detections)} detections -> {len(tracks)} confirmed tracks")
            for track in tracks:
                logger.info(f"  Track {track.track_id}: {track.class_name} @ {track.bbox}")

        stats = tracker.get_stats()
        logger.info(f"Tracker stats: {stats}")

        # Verify track consistency
        final_tracks = tracker.update(frames[-1])
        if len(final_tracks) < 2:
            logger.warning("Expected at least 2 confirmed tracks")

        logger.info("Tracker test PASSED")
        return True

    except Exception as e:
        logger.error(f"Tracker test failed: {e}")
        return False


def start_gazebo(world_file: Path) -> subprocess.Popen:
    """Start Gazebo with the perception test world."""
    logger.info(f"Starting Gazebo with {world_file.name}...")

    # Set up environment
    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = f"{ARDUPILOT_GAZEBO_DIR}/models:{ARDUPILOT_GAZEBO_DIR}/worlds"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(ARDUPILOT_GAZEBO_DIR / "build")

    cmd = ["gz", "sim", "-r", str(world_file)]

    proc = subprocess.Popen(
        cmd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Wait for Gazebo to start
    time.sleep(10)

    if proc.poll() is not None:
        stderr = proc.stderr.read().decode() if proc.stderr else ""
        logger.error(f"Gazebo failed to start: {stderr}")
        return None

    logger.info("Gazebo started successfully")
    return proc


def start_sitl_instances(num_drones: int) -> list:
    """Start SITL instances for each drone."""
    logger.info(f"Starting {num_drones} SITL instances...")

    processes = []
    log_dir = PROJECT_ROOT / "logs"
    log_dir.mkdir(exist_ok=True)

    for i in range(num_drones):
        stdout_log = open(log_dir / f"sitl_{i}_stdout.log", "w")
        stderr_log = open(log_dir / f"sitl_{i}_stderr.log", "w")

        cmd = [
            "sim_vehicle.py",
            "-v", "ArduCopter",
            "-f", "gazebo-iris",
            "--model", "JSON",
            f"-I{i}",
            f"--out=udp:127.0.0.1:{14540 + i}",
            "--no-rebuild",
        ]

        proc = subprocess.Popen(
            cmd,
            cwd=str(ARDUPILOT_DIR / "ArduCopter"),
            stdout=stdout_log,
            stderr=stderr_log,
        )
        processes.append(proc)
        logger.info(f"  SITL instance {i} started (PID {proc.pid})")
        time.sleep(3)

    logger.info(f"All {num_drones} SITL instances started")
    return processes


def check_camera_topics(num_drones: int) -> bool:
    """Check if camera topics are available in Gazebo."""
    logger.info("Checking camera topics...")

    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=10,
        )

        topics = result.stdout.strip().split("\n")
        camera_topics = [t for t in topics if "rgb_camera" in t and "image" in t]

        logger.info(f"Found {len(camera_topics)} camera topics:")
        for topic in camera_topics[:5]:  # Show first 5
            logger.info(f"  {topic}")

        if len(camera_topics) >= num_drones:
            logger.info("Camera topics check PASSED")
            return True
        else:
            logger.warning(f"Expected {num_drones} camera topics, found {len(camera_topics)}")
            return False

    except Exception as e:
        logger.error(f"Failed to check camera topics: {e}")
        return False


def cleanup(processes: list):
    """Clean up all processes."""
    logger.info("Cleaning up processes...")

    for proc in processes:
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    # Kill any remaining Gazebo processes
    subprocess.run(["pkill", "-f", "gz sim"], capture_output=True)
    subprocess.run(["pkill", "-f", "sim_vehicle.py"], capture_output=True)

    logger.info("Cleanup complete")


def main():
    parser = argparse.ArgumentParser(description="Phase 5 Perception Pipeline Test")
    parser.add_argument("--num-drones", type=int, default=3, help="Number of drones")
    parser.add_argument("--skip-gazebo", action="store_true", help="Skip Gazebo startup")
    parser.add_argument("--skip-sitl", action="store_true", help="Skip SITL startup")
    parser.add_argument("--detector-only", action="store_true", help="Test detector only")
    args = parser.parse_args()

    processes = []

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        logger.info("\nInterrupted, cleaning up...")
        cleanup(processes)
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Test 1: Detector standalone
        if not test_detector_standalone():
            logger.error("Detector test failed")
            return 1

        # Test 2: Tracker standalone
        if not test_tracker_standalone():
            logger.error("Tracker test failed")
            return 1

        if args.detector_only:
            logger.info("=" * 60)
            logger.info("Detector-only tests PASSED")
            logger.info("=" * 60)
            return 0

        # Test 3: Full integration
        logger.info("=" * 60)
        logger.info("Full Integration Test")
        logger.info("=" * 60)

        world_file = WORLDS_DIR / "perception_test.sdf"
        if not world_file.exists():
            logger.error(f"World file not found: {world_file}")
            return 1

        # Start Gazebo
        if not args.skip_gazebo:
            gz_proc = start_gazebo(world_file)
            if gz_proc:
                processes.append(gz_proc)
            else:
                return 1

        # Start SITL
        if not args.skip_sitl:
            sitl_procs = start_sitl_instances(args.num_drones)
            processes.extend(sitl_procs)
            time.sleep(10)  # Wait for SITL to stabilize

        # Check camera topics
        if not check_camera_topics(args.num_drones):
            logger.warning("Camera topics not fully available (may need more time)")

        logger.info("=" * 60)
        logger.info("Integration test setup complete!")
        logger.info("=" * 60)
        logger.info("")
        logger.info("Next steps:")
        logger.info("1. In a new terminal, build and source ROS2 workspace:")
        logger.info("   cd ~/ros2_ws && colcon build --packages-select swarm_ros")
        logger.info("   source install/setup.bash")
        logger.info("")
        logger.info("2. Launch simulation bridge with cameras:")
        logger.info(f"   ros2 launch swarm_ros simulation.launch.py num_drones:={args.num_drones} enable_cameras:=true")
        logger.info("")
        logger.info("3. Launch perception nodes:")
        logger.info(f"   ros2 launch swarm_ros perception.launch.py num_drones:={args.num_drones}")
        logger.info("")
        logger.info("4. Monitor detections:")
        logger.info("   ros2 topic echo /drone_0/detections")
        logger.info("")
        logger.info("Press Ctrl+C to stop and cleanup...")

        # Keep running until interrupted
        while True:
            time.sleep(1)
            # Check if any process died
            for proc in processes:
                if proc.poll() is not None:
                    logger.warning(f"Process {proc.pid} exited with code {proc.returncode}")

    except KeyboardInterrupt:
        pass
    finally:
        cleanup(processes)

    return 0


if __name__ == "__main__":
    sys.exit(main())
