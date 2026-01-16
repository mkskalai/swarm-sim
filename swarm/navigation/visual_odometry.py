"""Visual odometry using ORB features with optical flow fallback."""

import logging
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

from .config import VOConfig, CameraConfig

logger = logging.getLogger(__name__)


@dataclass
class FrameResult:
    """Result from processing a single frame."""

    keypoints: list  # List of cv2.KeyPoint
    descriptors: Optional[np.ndarray]  # ORB descriptors (Nx32 uint8)
    timestamp: float  # Frame timestamp in seconds
    frame_id: int  # Sequential frame ID


@dataclass
class MatchResult:
    """Result from matching two frames."""

    src_points: np.ndarray  # Matched points in source frame (Nx2)
    dst_points: np.ndarray  # Matched points in destination frame (Nx2)
    num_matches: int  # Number of matches
    match_ratio: float  # Ratio of matches to features


@dataclass
class MotionEstimate:
    """Estimated motion between frames."""

    R: np.ndarray  # 3x3 rotation matrix
    t: np.ndarray  # 3x1 translation (unit vector, scale unknown)
    inlier_ratio: float  # Ratio of RANSAC inliers
    num_inliers: int  # Number of inliers
    valid: bool  # Whether estimate is valid
    method: str  # "orb" or "flow"


class ORBMatcher:
    """ORB feature extraction and matching."""

    def __init__(self, config: VOConfig, camera_config: CameraConfig):
        self.config = config
        self.camera_config = camera_config
        self._orb: Optional[cv2.ORB] = None
        self._matcher: Optional[cv2.BFMatcher] = None
        self._prev_result: Optional[FrameResult] = None
        self._frame_count = 0
        self._initialized = False
        self._use_gpu = False

    def initialize(self) -> bool:
        """Initialize ORB detector and matcher."""
        try:
            # Try GPU first if enabled
            if self.config.use_gpu:
                try:
                    if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                        self._orb = cv2.cuda.ORB_create(
                            nfeatures=self.config.max_features
                        )
                        self._use_gpu = True
                        logger.info("ORB initialized with CUDA GPU acceleration")
                    else:
                        raise RuntimeError("No CUDA devices available")
                except (cv2.error, AttributeError, RuntimeError) as e:
                    logger.info(f"GPU not available ({e}), falling back to CPU")
                    self._use_gpu = False

            # CPU fallback
            if not self._use_gpu:
                self._orb = cv2.ORB_create(
                    nfeatures=self.config.max_features,
                    scaleFactor=1.2,
                    nlevels=8,
                    edgeThreshold=31,
                    firstLevel=0,
                    WTA_K=2,
                    patchSize=31,
                    fastThreshold=20,
                )
                logger.info("ORB initialized on CPU")

            # Brute-force matcher with Hamming distance (for binary descriptors)
            self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

            self._initialized = True
            return True

        except Exception as e:
            logger.error(f"Failed to initialize ORB: {e}")
            return False

    def process_frame(self, frame: np.ndarray, timestamp: float) -> Optional[FrameResult]:
        """Extract ORB features from frame.

        Args:
            frame: Grayscale or BGR image
            timestamp: Frame timestamp in seconds

        Returns:
            FrameResult with keypoints and descriptors, or None if failed
        """
        if not self._initialized:
            if not self.initialize():
                return None

        # Convert to grayscale if needed
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        try:
            if self._use_gpu:
                # GPU path
                gpu_frame = cv2.cuda_GpuMat(gray)
                keypoints, descriptors = self._orb.detectAndComputeAsync(
                    gpu_frame, None
                )
                keypoints = self._orb.convert(keypoints)
                descriptors = descriptors.download() if descriptors is not None else None
            else:
                # CPU path
                keypoints, descriptors = self._orb.detectAndCompute(gray, None)

            if keypoints is None or len(keypoints) < self.config.min_matches:
                logger.debug(
                    f"Insufficient features: {len(keypoints) if keypoints else 0}"
                )
                return None

            result = FrameResult(
                keypoints=keypoints,
                descriptors=descriptors,
                timestamp=timestamp,
                frame_id=self._frame_count,
            )
            self._frame_count += 1
            return result

        except Exception as e:
            logger.error(f"Error in ORB feature extraction: {e}")
            return None

    def match_frames(
        self, result1: FrameResult, result2: FrameResult
    ) -> Optional[MatchResult]:
        """Match features between two frames.

        Args:
            result1: Previous frame result
            result2: Current frame result

        Returns:
            MatchResult with matched point pairs, or None if failed
        """
        if result1.descriptors is None or result2.descriptors is None:
            return None

        try:
            # KNN matching with k=2 for ratio test
            matches = self._matcher.knnMatch(
                result1.descriptors, result2.descriptors, k=2
            )

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < self.config.match_ratio * n.distance:
                        good_matches.append(m)

            if len(good_matches) < self.config.min_matches:
                logger.debug(f"Insufficient matches after ratio test: {len(good_matches)}")
                return None

            # Extract matched point coordinates
            src_pts = np.float32(
                [result1.keypoints[m.queryIdx].pt for m in good_matches]
            )
            dst_pts = np.float32(
                [result2.keypoints[m.trainIdx].pt for m in good_matches]
            )

            return MatchResult(
                src_points=src_pts,
                dst_points=dst_pts,
                num_matches=len(good_matches),
                match_ratio=len(good_matches) / len(result1.keypoints),
            )

        except Exception as e:
            logger.error(f"Error in feature matching: {e}")
            return None

    def estimate_motion(self, match_result: MatchResult) -> Optional[MotionEstimate]:
        """Estimate relative motion from matched points.

        Args:
            match_result: Matched point pairs

        Returns:
            MotionEstimate with R, t, or None if failed
        """
        K = self.camera_config.K

        try:
            # Compute Essential matrix with RANSAC
            E, mask = cv2.findEssentialMat(
                match_result.src_points,
                match_result.dst_points,
                K,
                method=cv2.RANSAC,
                prob=self.config.ransac_confidence,
                threshold=self.config.ransac_threshold,
            )

            if E is None:
                logger.debug("Failed to compute Essential matrix")
                return None

            # Count inliers
            inliers = mask.ravel().astype(bool)
            num_inliers = np.sum(inliers)
            inlier_ratio = num_inliers / len(inliers)

            if num_inliers < self.config.min_matches:
                logger.debug(f"Insufficient inliers: {num_inliers}")
                return None

            # Recover pose from Essential matrix
            _, R, t, pose_mask = cv2.recoverPose(
                E,
                match_result.src_points,
                match_result.dst_points,
                K,
                mask=mask,
            )

            # t is a unit vector (scale is unknown in monocular VO)
            t = t.flatten()

            return MotionEstimate(
                R=R,
                t=t,
                inlier_ratio=inlier_ratio,
                num_inliers=num_inliers,
                valid=True,
                method="orb",
            )

        except Exception as e:
            logger.error(f"Error in motion estimation: {e}")
            return None


class OpticalFlowFallback:
    """Lucas-Kanade optical flow for low-texture scenarios."""

    def __init__(self, config: VOConfig, camera_config: CameraConfig):
        self.config = config
        self.camera_config = camera_config

        # Lucas-Kanade parameters
        self._lk_params = dict(
            winSize=(config.flow_win_size, config.flow_win_size),
            maxLevel=config.flow_max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )

        # Shi-Tomasi corner detection parameters
        self._feature_params = dict(
            maxCorners=config.flow_max_corners,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=7,
        )

        self._prev_gray: Optional[np.ndarray] = None
        self._prev_points: Optional[np.ndarray] = None

    def reset(self) -> None:
        """Reset tracker state."""
        self._prev_gray = None
        self._prev_points = None

    def track(
        self, prev_frame: np.ndarray, curr_frame: np.ndarray
    ) -> Optional[MatchResult]:
        """Track points using optical flow.

        Args:
            prev_frame: Previous grayscale frame
            curr_frame: Current grayscale frame

        Returns:
            MatchResult with tracked point pairs, or None if failed
        """
        # Convert to grayscale if needed
        if len(prev_frame.shape) == 3:
            prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        else:
            prev_gray = prev_frame

        if len(curr_frame.shape) == 3:
            curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
        else:
            curr_gray = curr_frame

        # Detect features in previous frame if needed
        if self._prev_gray is None or self._prev_points is None:
            self._prev_points = cv2.goodFeaturesToTrack(
                prev_gray, **self._feature_params
            )
            self._prev_gray = prev_gray

            if self._prev_points is None or len(self._prev_points) < 10:
                logger.debug("Insufficient features for optical flow")
                return None

        # Track points forward
        curr_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, curr_gray, self._prev_points, None, **self._lk_params
        )

        if curr_points is None:
            self.reset()
            return None

        # Filter valid tracks and reshape to (N, 2)
        status = status.flatten()
        good_prev = self._prev_points[status == 1].reshape(-1, 2)
        good_curr = curr_points[status == 1].reshape(-1, 2)

        # Filter by minimum displacement
        if len(good_prev) > 0 and len(good_curr) > 0:
            displacements = np.linalg.norm(good_curr - good_prev, axis=1)
            moving_mask = displacements > self.config.flow_min_displacement
            good_prev = good_prev[moving_mask]
            good_curr = good_curr[moving_mask]

        if len(good_prev) < 10:
            # Re-detect features
            self._prev_points = cv2.goodFeaturesToTrack(
                curr_gray, **self._feature_params
            )
            self._prev_gray = curr_gray
            logger.debug("Re-detecting features due to track loss")
            return None

        # Update for next iteration
        self._prev_gray = curr_gray
        self._prev_points = good_curr.reshape(-1, 1, 2)

        return MatchResult(
            src_points=good_prev.reshape(-1, 2),
            dst_points=good_curr.reshape(-1, 2),
            num_matches=len(good_prev),
            match_ratio=len(good_prev) / self.config.flow_max_corners,
        )

    def estimate_motion(self, match_result: MatchResult) -> Optional[MotionEstimate]:
        """Estimate motion from optical flow matches."""
        K = self.camera_config.K

        try:
            E, mask = cv2.findEssentialMat(
                match_result.src_points,
                match_result.dst_points,
                K,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0,
            )

            if E is None:
                return None

            inliers = mask.ravel().astype(bool)
            num_inliers = np.sum(inliers)

            if num_inliers < 8:
                return None

            _, R, t, _ = cv2.recoverPose(
                E, match_result.src_points, match_result.dst_points, K, mask=mask
            )

            return MotionEstimate(
                R=R,
                t=t.flatten(),
                inlier_ratio=num_inliers / len(inliers),
                num_inliers=num_inliers,
                valid=True,
                method="flow",
            )

        except Exception as e:
            logger.error(f"Error in optical flow motion estimation: {e}")
            return None


class ScaleEstimator:
    """Estimate and maintain metric scale for monocular VO."""

    def __init__(self, config: VOConfig):
        self.config = config
        self._scale = config.initial_scale
        self._altitude_ref: Optional[float] = None
        self._scale_history: list = []
        self._max_history = 50

    def initialize(self, altitude: float) -> None:
        """Initialize scale from known altitude.

        Args:
            altitude: Initial altitude in meters (positive up)
        """
        self._altitude_ref = altitude
        self._scale = 1.0
        self._scale_history.clear()
        logger.info(f"Scale estimator initialized with altitude: {altitude:.2f}m")

    def update_from_altitude(
        self, vo_delta_z: float, baro_delta_z: float
    ) -> float:
        """Update scale estimate from barometer altitude change.

        Args:
            vo_delta_z: VO vertical displacement (unit scale)
            baro_delta_z: Barometer vertical displacement (metric)

        Returns:
            Updated scale factor
        """
        if abs(vo_delta_z) < 1e-6:
            return self._scale

        # Compute scale from this measurement
        new_scale = abs(baro_delta_z / vo_delta_z)

        # Sanity check
        if new_scale < 0.01 or new_scale > 100.0:
            logger.warning(f"Rejecting unreasonable scale: {new_scale}")
            return self._scale

        # Exponential moving average filter
        self._scale = (
            self.config.scale_filter_alpha * new_scale
            + (1 - self.config.scale_filter_alpha) * self._scale
        )

        # Store history
        self._scale_history.append(self._scale)
        if len(self._scale_history) > self._max_history:
            self._scale_history.pop(0)

        return self._scale

    def correct_with_gps(self, vo_distance: float, gps_distance: float) -> float:
        """Correct scale when GPS returns.

        Args:
            vo_distance: Distance traveled according to VO (unit scale)
            gps_distance: Distance traveled according to GPS (metric)

        Returns:
            Corrected scale factor
        """
        if abs(vo_distance) < 1e-6:
            return self._scale

        correction_scale = gps_distance / vo_distance

        # Gradual correction
        self._scale = 0.5 * correction_scale + 0.5 * self._scale

        logger.info(f"Scale corrected with GPS: {self._scale:.4f}")
        return self._scale

    def apply_scale(self, translation: np.ndarray) -> np.ndarray:
        """Apply current scale to unit translation vector.

        Args:
            translation: Unit translation vector

        Returns:
            Scaled translation in meters
        """
        return translation * self._scale

    @property
    def scale(self) -> float:
        """Current scale estimate."""
        return self._scale

    @property
    def scale_confidence(self) -> float:
        """Confidence in scale estimate (0-1)."""
        if len(self._scale_history) < 5:
            return 0.3  # Low confidence initially

        # Compute variance of recent scale estimates
        recent = self._scale_history[-10:]
        variance = np.var(recent)

        # Higher variance = lower confidence
        confidence = 1.0 / (1.0 + variance * 10)
        return min(1.0, max(0.1, confidence))


class VisualOdometry:
    """Combined visual odometry with ORB primary and optical flow fallback."""

    def __init__(self, vo_config: VOConfig, camera_config: CameraConfig):
        self.config = vo_config
        self.camera_config = camera_config

        self._orb = ORBMatcher(vo_config, camera_config)
        self._flow = OpticalFlowFallback(vo_config, camera_config)
        self._scale = ScaleEstimator(vo_config)

        self._prev_frame: Optional[np.ndarray] = None
        self._prev_result: Optional[FrameResult] = None
        self._prev_timestamp: float = 0.0

        self._consecutive_failures = 0
        self._max_consecutive_failures = 5
        self._frame_count = 0

    def initialize(self) -> bool:
        """Initialize visual odometry pipeline."""
        return self._orb.initialize()

    def set_initial_altitude(self, altitude: float) -> None:
        """Set initial altitude for scale estimation."""
        self._scale.initialize(altitude)

    def process_frame(
        self, frame: np.ndarray, timestamp: float
    ) -> Optional[MotionEstimate]:
        """Process a frame and estimate motion from previous frame.

        Args:
            frame: BGR or grayscale image
            timestamp: Frame timestamp in seconds

        Returns:
            MotionEstimate with scaled motion, or None if failed
        """
        self._frame_count += 1

        # Skip frames if configured
        if self.config.frame_skip > 1 and self._frame_count % self.config.frame_skip != 0:
            return None

        # Check for stale frame
        if timestamp - self._prev_timestamp > self.config.max_frame_delay:
            logger.debug("Skipping stale frame")
            self._prev_frame = frame.copy()
            self._prev_timestamp = timestamp
            self._prev_result = None
            return None

        motion: Optional[MotionEstimate] = None

        # Try ORB first
        current_result = self._orb.process_frame(frame, timestamp)

        if current_result is not None and self._prev_result is not None:
            match_result = self._orb.match_frames(self._prev_result, current_result)
            if match_result is not None:
                motion = self._orb.estimate_motion(match_result)

        # Fallback to optical flow if ORB fails
        if motion is None and self._prev_frame is not None:
            logger.debug("ORB failed, trying optical flow fallback")
            match_result = self._flow.track(self._prev_frame, frame)
            if match_result is not None:
                motion = self._flow.estimate_motion(match_result)

        # Update state
        self._prev_frame = frame.copy()
        self._prev_timestamp = timestamp
        self._prev_result = current_result

        if motion is not None:
            # Apply scale to translation
            motion.t = self._scale.apply_scale(motion.t)
            self._consecutive_failures = 0
        else:
            self._consecutive_failures += 1
            if self._consecutive_failures >= self._max_consecutive_failures:
                logger.warning(
                    f"VO failed {self._consecutive_failures} consecutive frames"
                )
                self._flow.reset()

        return motion

    def update_scale_from_altitude(self, vo_delta_z: float, baro_delta_z: float) -> None:
        """Update scale from barometer reading."""
        self._scale.update_from_altitude(vo_delta_z, baro_delta_z)

    def correct_scale_with_gps(self, vo_distance: float, gps_distance: float) -> None:
        """Correct scale when GPS becomes available."""
        self._scale.correct_with_gps(vo_distance, gps_distance)

    @property
    def scale(self) -> float:
        """Current scale estimate."""
        return self._scale.scale

    @property
    def scale_confidence(self) -> float:
        """Confidence in scale estimate."""
        return self._scale.scale_confidence

    @property
    def is_healthy(self) -> bool:
        """Whether VO is producing valid estimates."""
        return self._consecutive_failures < self._max_consecutive_failures
