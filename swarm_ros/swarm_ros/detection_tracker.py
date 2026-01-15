"""
DetectionTracker - P2P detection cache and aggregation.

Similar to NeighborTracker, subscribes to /drone_N/detections
from all peers and maintains a cache for querying.

Usage:
    tracker = DetectionTracker(node, own_drone_id=0, num_drones=3)
    tracker.start()

    # Query detections
    all_detections = tracker.get_all_detections()
    nearby = tracker.get_detections_near(position, radius=20.0)
    people = tracker.get_detections_by_class("person")
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from rclpy.node import Node

logger = logging.getLogger(__name__)


@dataclass
class CachedDetection:
    """Cached detection from a peer drone."""

    drone_id: int
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]  # x, y, w, h
    estimated_position_ned: Optional[tuple[float, float, float]]
    position_confidence: float
    drone_position_ned: tuple[float, float, float]
    drone_yaw_deg: float
    timestamp: float  # Unix timestamp


@dataclass
class DetectionTrackerConfig:
    """Configuration for DetectionTracker."""

    cache_timeout: float = 5.0  # Seconds before detection expires
    max_detections_per_drone: int = 50
    position_merge_threshold: float = 3.0  # Meters for deduplication


class DetectionTracker:
    """
    Tracks detections from all peer drones.

    Provides:
    - Cache of recent detections from all drones
    - Spatial queries (detections near position)
    - Class-based filtering
    - Callbacks for new detections
    """

    def __init__(
        self,
        node: Node,
        own_drone_id: int,
        num_drones: int,
        config: Optional[DetectionTrackerConfig] = None,
    ):
        """
        Initialize DetectionTracker.

        Args:
            node: ROS2 node to attach subscriptions to
            own_drone_id: This drone's ID (0-indexed)
            num_drones: Total number of drones in swarm
            config: Optional configuration
        """
        self.node = node
        self.own_drone_id = own_drone_id
        self.num_drones = num_drones
        self.config = config or DetectionTrackerConfig()

        # Cache: drone_id -> list of CachedDetection
        self._cache: dict[int, list[CachedDetection]] = {}

        # Callbacks
        self._on_new_detection: list[Callable[[CachedDetection], None]] = []

        self._subscriptions = []
        self._started = False

    def start(self) -> None:
        """Start tracking by subscribing to peer detection topics."""
        if self._started:
            logger.warning("DetectionTracker already started")
            return

        try:
            from swarm_ros.msg import DetectionArray
        except ImportError as e:
            logger.error(f"swarm_ros messages not built: {e}")
            raise

        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        for drone_id in range(self.num_drones):
            topic = f"/drone_{drone_id}/detections"
            sub = self.node.create_subscription(
                DetectionArray,
                topic,
                lambda msg, did=drone_id: self._on_detections(did, msg),
                qos,
            )
            self._subscriptions.append(sub)
            self._cache[drone_id] = []

        self._started = True
        logger.info(f"DetectionTracker started, tracking {self.num_drones} drones")

    def stop(self) -> None:
        """Stop tracking and clean up subscriptions."""
        for sub in self._subscriptions:
            self.node.destroy_subscription(sub)
        self._subscriptions.clear()
        self._cache.clear()
        self._started = False
        logger.info("DetectionTracker stopped")

    def _on_detections(self, drone_id: int, msg) -> None:
        """Handle incoming DetectionArray from a drone."""
        now = time.time()

        new_detections = []
        for det in msg.detections:
            # Parse estimated position (check for NaN)
            pos = None
            if len(det.estimated_position_ned) == 3:
                if not any(math.isnan(x) for x in det.estimated_position_ned):
                    pos = tuple(det.estimated_position_ned)

            cached = CachedDetection(
                drone_id=det.drone_id,
                track_id=det.track_id,
                class_id=det.class_id,
                class_name=det.class_name,
                confidence=det.confidence,
                bbox=(det.bbox_x, det.bbox_y, det.bbox_width, det.bbox_height),
                estimated_position_ned=pos,
                position_confidence=det.position_confidence,
                drone_position_ned=tuple(msg.drone_position_ned),
                drone_yaw_deg=msg.drone_yaw_deg,
                timestamp=now,
            )
            new_detections.append(cached)

            # Fire callbacks
            for callback in self._on_new_detection:
                try:
                    callback(cached)
                except Exception as e:
                    logger.error(f"Detection callback error: {e}")

        # Update cache (replace all detections from this drone)
        self._cache[drone_id] = new_detections[: self.config.max_detections_per_drone]
        self._cleanup_cache()

    def _cleanup_cache(self) -> None:
        """Remove expired detections from all drones."""
        now = time.time()
        for drone_id in self._cache:
            self._cache[drone_id] = [
                d
                for d in self._cache[drone_id]
                if now - d.timestamp < self.config.cache_timeout
            ]

    def get_all_detections(self) -> list[CachedDetection]:
        """Get all cached detections from all drones."""
        self._cleanup_cache()
        all_dets = []
        for dets in self._cache.values():
            all_dets.extend(dets)
        return all_dets

    def get_detections_from_drone(self, drone_id: int) -> list[CachedDetection]:
        """Get detections from a specific drone."""
        self._cleanup_cache()
        return self._cache.get(drone_id, []).copy()

    def get_detections_from_peers(self) -> list[CachedDetection]:
        """Get detections from all peers (excluding own drone)."""
        self._cleanup_cache()
        all_dets = []
        for drone_id, dets in self._cache.items():
            if drone_id != self.own_drone_id:
                all_dets.extend(dets)
        return all_dets

    def get_detections_near(
        self,
        position: tuple[float, float, float],
        radius: float,
    ) -> list[CachedDetection]:
        """
        Get detections within radius of position.

        Args:
            position: Query position in NED frame (north, east, down)
            radius: Search radius in meters

        Returns:
            List of detections within radius (only those with position estimates)
        """
        result = []
        for det in self.get_all_detections():
            if det.estimated_position_ned is None:
                continue
            dx = det.estimated_position_ned[0] - position[0]
            dy = det.estimated_position_ned[1] - position[1]
            dz = det.estimated_position_ned[2] - position[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist <= radius:
                result.append(det)
        return result

    def get_detections_by_class(self, class_name: str) -> list[CachedDetection]:
        """Get all detections of a specific class."""
        return [d for d in self.get_all_detections() if d.class_name == class_name]

    def get_unique_tracks(self) -> dict[int, CachedDetection]:
        """
        Get unique tracks, preferring higher confidence.

        Returns:
            Dict mapping track_id to best CachedDetection
        """
        unique: dict[int, CachedDetection] = {}
        for det in self.get_all_detections():
            if det.track_id == 0:
                continue  # Skip untracked
            if det.track_id not in unique:
                unique[det.track_id] = det
            elif det.confidence > unique[det.track_id].confidence:
                unique[det.track_id] = det
        return unique

    def on_new_detection(
        self, callback: Callable[[CachedDetection], None]
    ) -> None:
        """
        Register callback for new detections.

        The callback will be called for each detection as it arrives.
        """
        self._on_new_detection.append(callback)

    def get_stats(self) -> dict:
        """Get tracker statistics."""
        self._cleanup_cache()
        total = sum(len(dets) for dets in self._cache.values())
        by_class: dict[str, int] = {}
        for det in self.get_all_detections():
            by_class[det.class_name] = by_class.get(det.class_name, 0) + 1

        return {
            "total_cached": total,
            "drones_reporting": sum(1 for dets in self._cache.values() if dets),
            "by_class": by_class,
            "started": self._started,
        }
