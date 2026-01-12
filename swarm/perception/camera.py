"""Camera interface for Gazebo RGB camera sensor.

Uses gz-transport (Gazebo Transport) to subscribe to camera images.
For ROS2 integration, use ros_gz_bridge to bridge topics.
"""

import asyncio
import logging
from dataclasses import dataclass
from typing import Callable, Optional
import subprocess
import threading
import queue

logger = logging.getLogger(__name__)


@dataclass
class CameraConfig:
    """Camera configuration."""

    topic: str = "/drone/camera"
    width: int = 640
    height: int = 480
    fps: float = 30.0


class GazeboCamera:
    """Interface to Gazebo camera sensor via gz-transport.

    This class provides methods to subscribe to camera images from
    Gazebo simulation. For actual image processing, the images can
    be bridged to ROS2 or processed directly.

    Note: Full gz-transport Python bindings require additional setup.
    This implementation provides the interface; actual subscription
    may require ROS2 bridge or direct gz-transport bindings.
    """

    def __init__(self, config: Optional[CameraConfig] = None):
        """Initialize camera interface.

        Args:
            config: Camera configuration. Uses defaults if not provided.
        """
        self.config = config or CameraConfig()
        self._is_running = False
        self._frame_queue: queue.Queue = queue.Queue(maxsize=10)
        self._callback: Optional[Callable] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def topic(self) -> str:
        """Gazebo transport topic for camera images."""
        return self.config.topic

    def get_ros2_bridge_command(self) -> str:
        """Get the ros2-gz-bridge command for this camera.

        Returns:
            Command string to bridge camera topic to ROS2.
        """
        return (
            f"ros2 run ros_gz_bridge parameter_bridge "
            f"{self.config.topic}@sensor_msgs/msg/Image@gz.msgs.Image"
        )

    def list_available_topics(self) -> list[str]:
        """List available Gazebo transport topics.

        Returns:
            List of topic names.
        """
        try:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                topics = [t.strip() for t in result.stdout.strip().split("\n") if t.strip()]
                return topics
            else:
                logger.error(f"Failed to list topics: {result.stderr}")
                return []
        except subprocess.TimeoutExpired:
            logger.error("Timeout listing Gazebo topics")
            return []
        except FileNotFoundError:
            logger.error("gz command not found - is Gazebo installed?")
            return []

    def echo_topic(self, num_messages: int = 1) -> Optional[str]:
        """Echo messages from the camera topic.

        Args:
            num_messages: Number of messages to capture.

        Returns:
            Raw message string or None if failed.
        """
        try:
            result = subprocess.run(
                ["gz", "topic", "-e", "-t", self.config.topic, "-n", str(num_messages)],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode == 0:
                return result.stdout
            else:
                logger.error(f"Failed to echo topic: {result.stderr}")
                return None
        except subprocess.TimeoutExpired:
            logger.warning(f"Timeout waiting for messages on {self.config.topic}")
            return None

    def is_publishing(self) -> bool:
        """Check if the camera topic is actively publishing.

        Returns:
            True if messages are being published.
        """
        topics = self.list_available_topics()
        return self.config.topic in topics

    async def wait_for_camera(self, timeout: float = 30.0) -> bool:
        """Wait for camera to become available.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if camera became available.
        """
        logger.info(f"Waiting for camera on {self.config.topic}...")

        start_time = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start_time < timeout:
            if self.is_publishing():
                logger.info("Camera topic found!")
                return True
            await asyncio.sleep(1.0)

        logger.error(f"Camera not found within {timeout}s")
        return False


def create_ros2_camera_node():
    """Template for ROS2 camera subscription node.

    This returns Python code that can be used to create a ROS2 node
    for receiving camera images. Requires ros2 jazzy environment.

    Returns:
        Python code string for ROS2 node.
    """
    return '''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    """ROS2 node for subscribing to drone camera."""

    def __init__(self):
        super().__init__("drone_camera_subscriber")
        self.subscription = self.create_subscription(
            Image,
            "/drone/camera",
            self.image_callback,
            10,
        )
        self.bridge = CvBridge()
        self.get_logger().info("Camera subscriber initialized")

    def image_callback(self, msg):
        """Process incoming camera image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image here
            cv2.imshow("Drone Camera", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
'''
