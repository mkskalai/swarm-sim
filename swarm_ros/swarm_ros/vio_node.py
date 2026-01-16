"""
VIONode - Visual-Inertial Odometry node for GPS-denied navigation.

Subscribes to camera images, runs visual odometry with IMU fusion,
and publishes VIO state estimates.

Usage:
    ros2 run swarm_ros vio_node --ros-args -p drone_id:=0
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# Add project root to path for swarm module access
# colcon copies files to install/, so we need to find the actual project location
_project_root = None

# Method 1: Check SWARM_PROJECT_ROOT environment variable
if os.environ.get("SWARM_PROJECT_ROOT"):
    _project_root = Path(os.environ["SWARM_PROJECT_ROOT"])

# Method 2: Follow ROS2 workspace symlink (src/swarm_ros -> actual project)
if _project_root is None or not (_project_root / "swarm").exists():
    _ros2_ws = Path.home() / "ros2_ws"
    _symlink = _ros2_ws / "src" / "swarm_ros"
    if _symlink.is_symlink():
        # swarm_ros symlink points to project/swarm_ros, so parent is project root
        _project_root = _symlink.resolve().parent

# Method 3: Check common project location
if _project_root is None or not (_project_root / "swarm").exists():
    _common_path = Path.home() / "Desktop" / "python projects" / "claude" / "swarm"
    if (_common_path / "swarm").exists():
        _project_root = _common_path

# Method 4: Check ~/swarm symlink (recommended in setup_env.sh)
if _project_root is None or not (_project_root / "swarm").exists():
    _swarm_link = Path.home() / "swarm"
    if _swarm_link.exists() and (_swarm_link / "swarm").exists():
        _project_root = _swarm_link.resolve()

if _project_root and (_project_root / "swarm").exists():
    if str(_project_root) not in sys.path:
        sys.path.insert(0, str(_project_root))

import logging
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

logger = logging.getLogger(__name__)


class VIONode(Node):
    """
    ROS2 node for Visual-Inertial Odometry.

    Subscribers:
        /drone_N/camera/image (Image) - Camera feed from Gazebo bridge
        /drone_N/state (DroneState) - Drone state for GPS reference

    Publishers:
        /drone_N/vio/state (VIOState) - VIO position/velocity estimate

    Services:
        /drone_N/vio/reset - Reset VIO to current GPS position
    """

    def __init__(self):
        super().__init__("vio_node")

        # Declare parameters
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("use_gpu", True)
        self.declare_parameter("navigation_mode", "hybrid")  # gps, vio, hybrid
        self.declare_parameter("publish_rate_limit", 15.0)  # Max Hz
        self.declare_parameter("mavlink_port", 14540)

        self.drone_id = self.get_parameter("drone_id").value
        self.get_logger().info(f"Initializing VIONode for drone {self.drone_id}")

        # Import navigation modules
        try:
            from swarm.navigation import (
                NavigationConfig,
                NavigationMode,
                VIOEstimator,
            )

            self._NavigationMode = NavigationMode
        except ImportError as e:
            self.get_logger().error(f"Failed to import swarm.navigation: {e}")
            raise

        # Import ROS2 messages
        try:
            from swarm_ros.msg import DroneState, VIOState

            self._DroneState = DroneState
            self._VIOState = VIOState
        except ImportError as e:
            self.get_logger().error(
                f"Failed to import swarm_ros messages: {e}. "
                "Did you build the workspace? (colcon build)"
            )
            raise

        # Import cv_bridge
        try:
            from cv_bridge import CvBridge

            self.bridge = CvBridge()
        except ImportError as e:
            self.get_logger().error(
                f"Failed to import cv_bridge: {e}. "
                "Install with: sudo apt install ros-jazzy-cv-bridge"
            )
            raise

        # Create navigation config
        mode_str = self.get_parameter("navigation_mode").value
        if mode_str == "gps":
            mode = NavigationMode.GPS
        elif mode_str == "vio":
            mode = NavigationMode.VIO
        else:
            mode = NavigationMode.HYBRID

        config = NavigationConfig.for_simulation()
        config.mode = mode
        config.vo.use_gpu = self.get_parameter("use_gpu").value

        # Initialize VIO estimator
        self.vio = VIOEstimator(config, drone_id=self.drone_id)

        self.get_logger().info("Initializing VIO estimator...")
        if not self.vio._vo.initialize():
            self.get_logger().warn(
                "VO initialization may have issues, will retry on first frame"
            )

        # State tracking
        self._initialized = False
        self._last_gps_position: Optional[np.ndarray] = None
        self._last_gps_velocity: Optional[np.ndarray] = None
        self._last_gps_time: float = 0.0
        self._last_publish_time: float = 0.0
        self._rate_limit = 1.0 / self.get_parameter("publish_rate_limit").value
        self._frame_seq = 0

        # QoS profiles
        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            f"/drone_{self.drone_id}/camera/image",
            self._on_image,
            best_effort_qos,
        )

        # Subscribe to drone state (for GPS reference)
        self.state_sub = self.create_subscription(
            self._DroneState,
            f"/drone_{self.drone_id}/state",
            self._on_drone_state,
            best_effort_qos,
        )

        # Publisher for VIO state
        self.vio_pub = self.create_publisher(
            self._VIOState,
            f"/drone_{self.drone_id}/vio/state",
            reliable_qos,
        )

        self.get_logger().info(
            f"VIONode initialized for drone {self.drone_id} "
            f"(mode: {mode_str}, GPU: {config.vo.use_gpu})"
        )

    def _on_drone_state(self, msg) -> None:
        """Handle drone state message (GPS reference)."""
        # Extract position from DroneState
        self._last_gps_position = np.array(msg.position_ned, dtype=np.float64)
        self._last_gps_velocity = np.array(msg.velocity_ned, dtype=np.float64)
        self._last_gps_time = time.time()

        # Initialize VIO on first GPS fix
        if not self._initialized and msg.ekf_ok:
            yaw_rad = np.radians(msg.yaw_deg)
            attitude = np.array([0.0, 0.0, yaw_rad])
            altitude = -self._last_gps_position[2]  # Down to up

            timestamp_us = int(time.time() * 1e6)

            success = self.vio.initialize(
                position=self._last_gps_position,
                velocity=self._last_gps_velocity,
                attitude=attitude,
                altitude=altitude,
                timestamp_us=timestamp_us,
            )

            if success:
                self._initialized = True
                self.get_logger().info(
                    f"VIO initialized at position "
                    f"({self._last_gps_position[0]:.1f}, "
                    f"{self._last_gps_position[1]:.1f}, "
                    f"{self._last_gps_position[2]:.1f})"
                )
        elif self._initialized:
            # Update VIO with GPS for fusion
            self.vio.update_with_gps(
                self._last_gps_position,
                self._last_gps_velocity,
                self._last_gps_time,
            )

    def _on_image(self, msg: Image) -> None:
        """Handle camera image message."""
        if not self._initialized:
            return

        # Rate limiting
        now = time.time()
        if now - self._last_publish_time < self._rate_limit:
            return

        # Convert ROS image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get timestamp
        timestamp_us = int(
            msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000
        )

        # Process frame
        try:
            state = self.vio.process_image(cv_image, timestamp_us)
        except Exception as e:
            self.get_logger().error(f"VIO processing error: {e}")
            return

        if state is None:
            return

        # Publish VIO state
        self._publish_vio_state(state)
        self._last_publish_time = now
        self._frame_seq += 1

    def _publish_vio_state(self, state) -> None:
        """Publish VIO state message."""
        msg = self._VIOState()

        msg.drone_id = self.drone_id

        # Position and velocity
        msg.position_ned = list(state.position)
        msg.velocity_ned = list(state.velocity)

        # Orientation
        msg.orientation = list(state.quaternion)

        # Uncertainties
        msg.position_std = list(state.position_std)
        msg.velocity_std = list(state.velocity_std)

        # Mode mapping
        mode_map = {
            self._NavigationMode.GPS: msg.MODE_GPS,
            self._NavigationMode.VIO: msg.MODE_VIO,
            self._NavigationMode.HYBRID: msg.MODE_HYBRID,
            self._NavigationMode.DEAD_RECKONING: msg.MODE_DEAD_RECKONING,
        }
        msg.mode = mode_map.get(state.mode, msg.MODE_DEAD_RECKONING)

        # Quality metrics
        msg.vo_quality = float(state.vo_quality)
        msg.scale_confidence = float(state.scale_confidence)
        msg.imu_sample_count = state.imu_sample_count
        msg.scale = float(self.vio.scale)
        msg.imu_rate = float(self.vio.imu_rate)

        # Timestamps
        msg.stamp = self.get_clock().now().to_msg()

        # Last VO time (from state timestamp)
        vo_sec = state.timestamp_us // 1_000_000
        vo_nsec = (state.timestamp_us % 1_000_000) * 1000
        msg.last_vo_time.sec = vo_sec
        msg.last_vo_time.nanosec = vo_nsec

        # Last GPS time
        if self._last_gps_time > 0:
            gps_sec = int(self._last_gps_time)
            gps_nsec = int((self._last_gps_time - gps_sec) * 1e9)
            msg.last_gps_time.sec = gps_sec
            msg.last_gps_time.nanosec = gps_nsec

        self.vio_pub.publish(msg)

    def destroy_node(self) -> None:
        """Clean up resources."""
        self.get_logger().info("Shutting down VIONode")
        self.vio.stop_imu_handler()
        super().destroy_node()


def main(args=None):
    """Entry point for VIO node."""
    rclpy.init(args=args)

    node = VIONode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
