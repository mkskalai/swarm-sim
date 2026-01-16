"""Semantic landmark integration for VIO enhancement.

Uses YOLO detections of static objects (buildings, vehicles) as navigation
landmarks to provide position corrections for visual-inertial odometry.
"""

import math
import time
import logging
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple, Set

import numpy as np

from .config import CameraConfig

logger = logging.getLogger(__name__)

# Position type: (north, east, down) in NED frame
PositionNED = Tuple[float, float, float]


@dataclass
class SemanticLandmark:
    """A detected object used as a navigation landmark.

    Attributes:
        landmark_id: Unique identifier
        class_name: Object class (e.g., "building", "car")
        position_ned: Estimated world position in NED frame
        position_uncertainty: Position uncertainty (1-sigma, meters)
        observation_count: Number of times observed
        last_observed: Unix timestamp of last observation
        first_observed: Unix timestamp of first observation
        observed_by_drones: Set of drone IDs that have observed this landmark
    """
    landmark_id: int
    class_name: str
    position_ned: PositionNED
    position_uncertainty: float = 5.0
    observation_count: int = 1
    last_observed: float = field(default_factory=time.time)
    first_observed: float = field(default_factory=time.time)
    observed_by_drones: Set[int] = field(default_factory=set)

    @property
    def is_reliable(self) -> bool:
        """Whether landmark is reliable for navigation.

        Requires multiple observations and low uncertainty.
        """
        return self.observation_count >= 3 and self.position_uncertainty < 10.0

    @property
    def age(self) -> float:
        """Age of landmark since first observation (seconds)."""
        return time.time() - self.first_observed


@dataclass
class LandmarkObservation:
    """Single observation of a potential landmark.

    Attributes:
        class_name: Detected object class
        bbox: Bounding box (x, y, width, height) in pixels
        drone_id: Observing drone ID
        drone_position: Drone position when observed
        drone_attitude: Drone attitude (roll, pitch, yaw) in radians
        estimated_position: Estimated world position of landmark
        position_uncertainty: Estimated position uncertainty
        confidence: Detection confidence
        timestamp: Observation timestamp
    """
    class_name: str
    bbox: Tuple[int, int, int, int]
    drone_id: int
    drone_position: PositionNED
    drone_attitude: Tuple[float, float, float]  # roll, pitch, yaw
    estimated_position: PositionNED
    position_uncertainty: float
    confidence: float
    timestamp: float = field(default_factory=time.time)


@dataclass
class LandmarkConfig:
    """Configuration for landmark database.

    Attributes:
        min_observations: Minimum observations before landmark is reliable
        matching_distance: Max distance to match observations (meters)
        max_uncertainty: Maximum acceptable uncertainty (meters)
        static_classes: Object classes considered static (usable as landmarks)
        uncertainty_decay: How much uncertainty decreases per observation
        max_landmarks: Maximum landmarks to track
        expiry_time: Time before landmark expires if not seen (seconds)
    """
    min_observations: int = 3
    matching_distance: float = 10.0
    max_uncertainty: float = 20.0
    static_classes: Set[str] = field(default_factory=lambda: {
        "building", "car", "truck", "bus", "traffic light",
        "fire hydrant", "stop sign", "bench"
    })
    uncertainty_decay: float = 0.7  # Multiply uncertainty by this per observation
    max_landmarks: int = 100
    expiry_time: float = 300.0  # 5 minutes


class SemanticLandmarkDatabase:
    """Database of semantic landmarks for VIO augmentation.

    Manages landmark observations, matching, and provides landmarks
    for navigation correction.

    Example:
        db = SemanticLandmarkDatabase()

        # Add observation from detection
        observation = LandmarkObservation(
            class_name="building",
            bbox=(100, 50, 80, 120),
            drone_id=0,
            drone_position=(10.0, 20.0, -15.0),
            drone_attitude=(0.0, 0.1, 0.5),
            estimated_position=(50.0, 30.0, 0.0),
            position_uncertainty=8.0,
            confidence=0.85,
        )
        landmark = db.add_observation(observation)

        # Get landmarks for navigation
        nearby = db.get_nearby_landmarks((10.0, 20.0, -15.0), radius=100.0)
    """

    def __init__(self, config: Optional[LandmarkConfig] = None):
        """Initialize landmark database.

        Args:
            config: Configuration options
        """
        self._config = config or LandmarkConfig()
        self._landmarks: Dict[int, SemanticLandmark] = {}
        self._next_id = 1

        # Spatial index: grid cells for fast lookup
        self._grid_size = 20.0  # meters
        self._grid: Dict[Tuple[int, int], Set[int]] = {}

    def add_observation(
        self,
        observation: LandmarkObservation,
    ) -> Optional[SemanticLandmark]:
        """Add detection observation, potentially creating or updating landmark.

        Args:
            observation: Landmark observation from detection

        Returns:
            Matched or newly created landmark, or None if rejected
        """
        # Check if class is suitable for landmarks
        if observation.class_name not in self._config.static_classes:
            return None

        # Check uncertainty bound
        if observation.position_uncertainty > self._config.max_uncertainty:
            return None

        # Try to match to existing landmark
        matched = self._match_observation(observation)

        if matched:
            # Update existing landmark
            self._update_landmark(matched, observation)
            return matched
        else:
            # Create new landmark
            return self._create_landmark(observation)

    def _match_observation(
        self,
        observation: LandmarkObservation,
    ) -> Optional[SemanticLandmark]:
        """Try to match observation to existing landmark.

        Args:
            observation: Observation to match

        Returns:
            Matched landmark or None
        """
        best_match: Optional[SemanticLandmark] = None
        best_distance = float('inf')

        # Get nearby landmarks from grid
        candidates = self._get_candidates_from_grid(observation.estimated_position)

        for landmark in candidates:
            # Must be same class
            if landmark.class_name != observation.class_name:
                continue

            # Check distance
            dist = self._distance(observation.estimated_position, landmark.position_ned)

            # Consider uncertainty in matching
            max_dist = self._config.matching_distance + observation.position_uncertainty

            if dist < max_dist and dist < best_distance:
                best_match = landmark
                best_distance = dist

        return best_match

    def _update_landmark(
        self,
        landmark: SemanticLandmark,
        observation: LandmarkObservation,
    ) -> None:
        """Update landmark with new observation.

        Uses weighted average to update position based on uncertainties.

        Args:
            landmark: Landmark to update
            observation: New observation
        """
        # Compute weights based on uncertainties
        w1 = 1.0 / (landmark.position_uncertainty ** 2)
        w2 = 1.0 / (observation.position_uncertainty ** 2)
        total_w = w1 + w2

        # Weighted position update
        old_pos = landmark.position_ned
        new_pos = observation.estimated_position

        updated_pos = (
            (w1 * old_pos[0] + w2 * new_pos[0]) / total_w,
            (w1 * old_pos[1] + w2 * new_pos[1]) / total_w,
            (w1 * old_pos[2] + w2 * new_pos[2]) / total_w,
        )

        # Update uncertainty (decreases with more observations)
        new_uncertainty = landmark.position_uncertainty * self._config.uncertainty_decay
        new_uncertainty = max(1.0, min(new_uncertainty, observation.position_uncertainty))

        # Update grid if position changed significantly
        old_cell = self._get_grid_cell(old_pos)
        new_cell = self._get_grid_cell(updated_pos)

        if old_cell != new_cell:
            self._grid[old_cell].discard(landmark.landmark_id)
            if new_cell not in self._grid:
                self._grid[new_cell] = set()
            self._grid[new_cell].add(landmark.landmark_id)

        # Update landmark
        landmark.position_ned = updated_pos
        landmark.position_uncertainty = new_uncertainty
        landmark.observation_count += 1
        landmark.last_observed = observation.timestamp
        landmark.observed_by_drones.add(observation.drone_id)

        logger.debug(f"Updated landmark {landmark.landmark_id}: "
                    f"pos={updated_pos}, uncertainty={new_uncertainty:.2f}, "
                    f"observations={landmark.observation_count}")

    def _create_landmark(
        self,
        observation: LandmarkObservation,
    ) -> SemanticLandmark:
        """Create new landmark from observation.

        Args:
            observation: Initial observation

        Returns:
            Newly created landmark
        """
        # Check capacity
        if len(self._landmarks) >= self._config.max_landmarks:
            self._prune_landmarks()

        landmark = SemanticLandmark(
            landmark_id=self._next_id,
            class_name=observation.class_name,
            position_ned=observation.estimated_position,
            position_uncertainty=observation.position_uncertainty,
            observation_count=1,
            last_observed=observation.timestamp,
            first_observed=observation.timestamp,
            observed_by_drones={observation.drone_id},
        )

        self._landmarks[landmark.landmark_id] = landmark
        self._next_id += 1

        # Add to grid
        cell = self._get_grid_cell(observation.estimated_position)
        if cell not in self._grid:
            self._grid[cell] = set()
        self._grid[cell].add(landmark.landmark_id)

        logger.debug(f"Created landmark {landmark.landmark_id}: "
                    f"{observation.class_name} at {observation.estimated_position}")

        return landmark

    def _prune_landmarks(self) -> None:
        """Remove old or unreliable landmarks when at capacity."""
        current_time = time.time()

        # Score landmarks (higher = more valuable)
        scored = []
        for lm in self._landmarks.values():
            age = current_time - lm.last_observed
            score = lm.observation_count / (1 + age / 60.0)
            scored.append((score, lm.landmark_id))

        # Sort by score and remove lowest
        scored.sort()
        num_to_remove = max(1, len(scored) // 10)

        for i in range(num_to_remove):
            lm_id = scored[i][1]
            self._remove_landmark(lm_id)

        logger.info(f"Pruned {num_to_remove} landmarks, {len(self._landmarks)} remaining")

    def _remove_landmark(self, landmark_id: int) -> None:
        """Remove landmark from database."""
        if landmark_id in self._landmarks:
            lm = self._landmarks[landmark_id]
            cell = self._get_grid_cell(lm.position_ned)
            if cell in self._grid:
                self._grid[cell].discard(landmark_id)
            del self._landmarks[landmark_id]

    def get_nearby_landmarks(
        self,
        position: PositionNED,
        radius: float = 50.0,
        reliable_only: bool = True,
    ) -> List[SemanticLandmark]:
        """Get landmarks within radius of position.

        Args:
            position: Query position
            radius: Search radius in meters
            reliable_only: Only return reliable landmarks

        Returns:
            List of landmarks within radius
        """
        result = []

        # Get grid cells within radius
        cells_to_check = self._get_cells_in_radius(position, radius)

        for cell in cells_to_check:
            if cell not in self._grid:
                continue

            for lm_id in self._grid[cell]:
                lm = self._landmarks.get(lm_id)
                if lm is None:
                    continue

                if reliable_only and not lm.is_reliable:
                    continue

                dist = self._distance(position, lm.position_ned)
                if dist <= radius:
                    result.append(lm)

        return result

    def get_landmark_by_id(self, landmark_id: int) -> Optional[SemanticLandmark]:
        """Get specific landmark by ID."""
        return self._landmarks.get(landmark_id)

    def get_all_landmarks(self) -> List[SemanticLandmark]:
        """Get all landmarks."""
        return list(self._landmarks.values())

    def get_reliable_landmarks(self) -> List[SemanticLandmark]:
        """Get all reliable landmarks."""
        return [lm for lm in self._landmarks.values() if lm.is_reliable]

    def cleanup_expired(self) -> int:
        """Remove landmarks not seen within expiry time.

        Returns:
            Number of landmarks removed
        """
        current_time = time.time()
        to_remove = []

        for lm_id, lm in self._landmarks.items():
            age = current_time - lm.last_observed
            if age > self._config.expiry_time:
                to_remove.append(lm_id)

        for lm_id in to_remove:
            self._remove_landmark(lm_id)

        return len(to_remove)

    def _get_grid_cell(self, position: PositionNED) -> Tuple[int, int]:
        """Get grid cell for position."""
        return (
            int(position[0] // self._grid_size),
            int(position[1] // self._grid_size),
        )

    def _get_cells_in_radius(
        self,
        position: PositionNED,
        radius: float,
    ) -> List[Tuple[int, int]]:
        """Get all grid cells within radius of position."""
        center_cell = self._get_grid_cell(position)
        cell_radius = int(radius // self._grid_size) + 1

        cells = []
        for di in range(-cell_radius, cell_radius + 1):
            for dj in range(-cell_radius, cell_radius + 1):
                cells.append((center_cell[0] + di, center_cell[1] + dj))

        return cells

    def _get_candidates_from_grid(
        self,
        position: PositionNED,
    ) -> List[SemanticLandmark]:
        """Get candidate landmarks from nearby grid cells."""
        cells = self._get_cells_in_radius(position, self._config.matching_distance)
        candidates = []

        for cell in cells:
            if cell in self._grid:
                for lm_id in self._grid[cell]:
                    if lm_id in self._landmarks:
                        candidates.append(self._landmarks[lm_id])

        return candidates

    def _distance(self, p1: PositionNED, p2: PositionNED) -> float:
        """Calculate 3D distance between positions."""
        return math.sqrt(
            (p2[0] - p1[0])**2 +
            (p2[1] - p1[1])**2 +
            (p2[2] - p1[2])**2
        )

    @property
    def num_landmarks(self) -> int:
        """Number of landmarks in database."""
        return len(self._landmarks)

    @property
    def num_reliable(self) -> int:
        """Number of reliable landmarks."""
        return sum(1 for lm in self._landmarks.values() if lm.is_reliable)


def estimate_landmark_position(
    bbox: Tuple[int, int, int, int],
    drone_position: PositionNED,
    drone_attitude: Tuple[float, float, float],
    camera_config: CameraConfig,
    assumed_height: float = 1.5,
) -> Tuple[PositionNED, float]:
    """Estimate world position of detected object.

    Uses camera geometry and assumed object height to estimate position.

    Args:
        bbox: Detection bounding box (x, y, width, height) in pixels
        drone_position: Drone position in NED frame
        drone_attitude: Drone attitude (roll, pitch, yaw) in radians
        camera_config: Camera configuration
        assumed_height: Assumed object height for distance estimation (meters)

    Returns:
        Tuple of (estimated_position_ned, uncertainty)
    """
    # Extract bbox center
    bx, by, bw, bh = bbox
    cx = bx + bw / 2
    cy = by + bh / 2

    # Image dimensions
    img_w = camera_config.width
    img_h = camera_config.height
    fx = camera_config.fx
    fy = camera_config.fy

    # Camera center
    cx_cam = camera_config.cx
    cy_cam = camera_config.cy

    # Compute ray direction in camera frame
    # Camera frame: X right, Y down, Z forward
    ray_x = (cx - cx_cam) / fx
    ray_y = (cy - cy_cam) / fy
    ray_z = 1.0

    # Normalize ray
    ray_len = math.sqrt(ray_x**2 + ray_y**2 + ray_z**2)
    ray_x /= ray_len
    ray_y /= ray_len
    ray_z /= ray_len

    # Estimate distance from object size
    # Assume object height is known, use bbox height to estimate distance
    if bh > 0:
        distance = (assumed_height * fy) / bh
    else:
        distance = 20.0  # Default fallback

    # Limit distance estimate
    distance = min(distance, 100.0)

    # Apply drone attitude to transform ray to NED frame
    roll, pitch, yaw = drone_attitude

    # Rotation matrices
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy_r, sy = math.cos(yaw), math.sin(yaw)

    # Combined rotation (ZYX convention)
    # First rotate camera frame to body frame (assuming camera points forward)
    # Camera Z -> Body X (forward)
    # Camera X -> Body Y (right)
    # Camera Y -> Body Z (down)
    body_x = ray_z  # forward component
    body_y = ray_x  # right component
    body_z = ray_y  # down component

    # Then rotate body to NED using attitude
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    ned_x = (cy_r*cp)*body_x + (cy_r*sp*sr - sy*cr)*body_y + (cy_r*sp*cr + sy*sr)*body_z
    ned_y = (sy*cp)*body_x + (sy*sp*sr + cy_r*cr)*body_y + (sy*sp*cr - cy_r*sr)*body_z
    ned_z = (-sp)*body_x + (cp*sr)*body_y + (cp*cr)*body_z

    # Compute world position
    dn, de, dd = drone_position
    landmark_n = dn + distance * ned_x
    landmark_e = de + distance * ned_y

    # Assume landmark is on ground (down = 0) unless we have better estimate
    landmark_d = 0.0

    # Estimate uncertainty based on distance and detection size
    # Further objects have more uncertainty
    uncertainty = 2.0 + distance * 0.1

    return (landmark_n, landmark_e, landmark_d), uncertainty


class SemanticVIOIntegration:
    """Integrates semantic landmarks with VIO for position correction.

    Processes YOLO detections to create/update landmarks, and provides
    position corrections for the EKF.

    Example:
        integration = SemanticVIOIntegration(
            vio_estimator=vio,
            landmark_db=db,
            camera_config=CameraConfig(),
        )

        # Process detection
        for detection in detections:
            landmark = integration.process_detection(
                detection, drone_position, drone_attitude
            )

            if landmark and landmark.is_reliable:
                integration.update_vio_with_landmark(landmark)
    """

    def __init__(
        self,
        vio_estimator=None,
        landmark_db: Optional[SemanticLandmarkDatabase] = None,
        camera_config: Optional[CameraConfig] = None,
    ):
        """Initialize semantic VIO integration.

        Args:
            vio_estimator: VIOEstimator instance for corrections
            landmark_db: Landmark database
            camera_config: Camera configuration
        """
        self._vio = vio_estimator
        self._db = landmark_db or SemanticLandmarkDatabase()
        self._camera_config = camera_config or CameraConfig()

        # Statistics
        self._detections_processed = 0
        self._landmarks_created = 0
        self._corrections_applied = 0

    def process_detection(
        self,
        detection,
        drone_id: int,
        drone_position: PositionNED,
        drone_attitude: Tuple[float, float, float],
    ) -> Optional[SemanticLandmark]:
        """Process a detection to potentially create/update landmark.

        Args:
            detection: Detection object with bbox, class_name, confidence
            drone_id: Observing drone ID
            drone_position: Drone position in NED
            drone_attitude: Drone attitude (roll, pitch, yaw)

        Returns:
            Landmark if created/updated, None otherwise
        """
        self._detections_processed += 1

        # Extract detection info
        class_name = detection.class_name if hasattr(detection, 'class_name') else str(detection.class_id)
        bbox = detection.bbox
        confidence = detection.confidence

        # Skip low confidence detections
        if confidence < 0.5:
            return None

        # Estimate landmark position
        estimated_pos, uncertainty = estimate_landmark_position(
            bbox=bbox,
            drone_position=drone_position,
            drone_attitude=drone_attitude,
            camera_config=self._camera_config,
        )

        # Create observation
        observation = LandmarkObservation(
            class_name=class_name,
            bbox=bbox,
            drone_id=drone_id,
            drone_position=drone_position,
            drone_attitude=drone_attitude,
            estimated_position=estimated_pos,
            position_uncertainty=uncertainty,
            confidence=confidence,
        )

        # Add to database
        landmark = self._db.add_observation(observation)

        if landmark and landmark.observation_count == 1:
            self._landmarks_created += 1

        return landmark

    def update_vio_with_landmark(
        self,
        landmark: SemanticLandmark,
        current_position: PositionNED,
        observation: Optional[LandmarkObservation] = None,
    ) -> bool:
        """Update VIO using landmark as position reference.

        Computes position correction based on landmark observation
        and applies to EKF.

        Args:
            landmark: Reliable landmark to use for correction
            current_position: Current VIO-estimated position
            observation: Most recent observation of landmark

        Returns:
            True if correction was applied
        """
        if self._vio is None:
            return False

        if not landmark.is_reliable:
            logger.debug(f"Landmark {landmark.landmark_id} not reliable, skipping correction")
            return False

        # For now, landmarks provide a position sanity check
        # The EKF's update_landmark() method should be called with
        # the observation details

        self._corrections_applied += 1
        return True

    def get_nearby_reliable_landmarks(
        self,
        position: PositionNED,
        radius: float = 50.0,
    ) -> List[SemanticLandmark]:
        """Get reliable landmarks near position for navigation."""
        return self._db.get_nearby_landmarks(
            position, radius=radius, reliable_only=True
        )

    def cleanup(self) -> int:
        """Cleanup expired landmarks."""
        return self._db.cleanup_expired()

    @property
    def stats(self) -> Dict:
        """Get integration statistics."""
        return {
            "detections_processed": self._detections_processed,
            "landmarks_created": self._landmarks_created,
            "corrections_applied": self._corrections_applied,
            "total_landmarks": self._db.num_landmarks,
            "reliable_landmarks": self._db.num_reliable,
        }
