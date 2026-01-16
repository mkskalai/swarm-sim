"""Visual-Inertial Odometry estimator combining VO, IMU, and EKF."""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .config import NavigationConfig, NavigationMode
from .ekf import NavigationEKF, EKFState
from .imu_handler import IMUHandler, IMUSample
from .visual_odometry import VisualOdometry, MotionEstimate
from .utils import (
    euler_from_quaternion,
    quaternion_from_euler,
    rotation_matrix_from_quaternion,
)

logger = logging.getLogger(__name__)


@dataclass
class VIOState:
    """Complete VIO state estimate."""

    # Position in NED frame (meters)
    position: np.ndarray

    # Velocity in NED frame (m/s)
    velocity: np.ndarray

    # Orientation as quaternion [w, x, y, z]
    quaternion: np.ndarray

    # Position uncertainty (1-sigma, meters)
    position_std: np.ndarray

    # Velocity uncertainty (1-sigma, m/s)
    velocity_std: np.ndarray

    # Navigation mode
    mode: NavigationMode

    # Timestamp in microseconds
    timestamp_us: int

    # Quality metrics
    vo_quality: float  # 0-1, based on feature matches
    scale_confidence: float  # 0-1, confidence in metric scale
    imu_sample_count: int  # IMU samples since last VO update

    @property
    def attitude_euler(self) -> Tuple[float, float, float]:
        """Get attitude as (roll, pitch, yaw) in radians."""
        return euler_from_quaternion(self.quaternion)

    @property
    def yaw_deg(self) -> float:
        """Get yaw angle in degrees."""
        _, _, yaw = self.attitude_euler
        return np.degrees(yaw)

    @property
    def position_ned(self) -> Tuple[float, float, float]:
        """Position as tuple (north, east, down)."""
        return tuple(self.position)

    @property
    def velocity_ned(self) -> Tuple[float, float, float]:
        """Velocity as tuple (vn, ve, vd)."""
        return tuple(self.velocity)


class VIOEstimator:
    """Visual-Inertial Odometry estimator.

    Orchestrates visual odometry, IMU handling, and EKF fusion to produce
    state estimates for GPS-denied navigation.
    """

    def __init__(self, config: NavigationConfig, drone_id: int = 0):
        """Initialize VIO estimator.

        Args:
            config: Navigation configuration
            drone_id: Drone identifier for logging
        """
        self.config = config
        self.drone_id = drone_id

        # Components
        self._vo = VisualOdometry(config.vo, config.camera)
        self._imu = IMUHandler(config.imu)
        self._ekf = NavigationEKF(config.ekf)

        # State
        self._initialized = False
        self._last_vo_timestamp_us: int = 0
        self._last_gps_timestamp: float = 0.0
        self._imu_samples_since_vo: int = 0

        # Accumulated motion for scale estimation
        self._accumulated_vo_distance = 0.0
        self._accumulated_gps_distance = 0.0
        self._last_gps_position: Optional[np.ndarray] = None

        # Thread safety
        self._lock = threading.Lock()

        # Performance metrics
        self._vo_success_rate = 1.0
        self._vo_attempts = 0
        self._vo_successes = 0

    def initialize(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        attitude: np.ndarray,
        altitude: float,
        timestamp_us: int,
    ) -> bool:
        """Initialize VIO estimator with known state.

        Args:
            position: Initial position [north, east, down] in meters
            velocity: Initial velocity [vn, ve, vd] in m/s
            attitude: Initial attitude as Euler [roll, pitch, yaw] or quaternion
            altitude: Initial altitude for scale estimation (positive up)
            timestamp_us: Initial timestamp in microseconds

        Returns:
            True if initialized successfully
        """
        with self._lock:
            # Convert attitude to quaternion if needed
            if len(attitude) == 3:
                quaternion = quaternion_from_euler(attitude[0], attitude[1], attitude[2])
            else:
                quaternion = attitude

            # Initialize VO
            if not self._vo.initialize():
                logger.error("Failed to initialize visual odometry")
                return False

            # Set initial altitude for scale
            self._vo.set_initial_altitude(altitude)

            # Initialize EKF
            self._ekf.initialize(
                position=position,
                velocity=velocity,
                quaternion=quaternion,
                timestamp_us=timestamp_us,
            )

            self._last_vo_timestamp_us = timestamp_us
            self._initialized = True

            logger.info(
                f"VIO estimator initialized for drone {self.drone_id} "
                f"at position ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f})"
            )
            return True

    def set_mavlink_connection(self, conn) -> None:
        """Set MAVLink connection for IMU reading.

        Args:
            conn: pymavlink mavutil.mavlink_connection object
        """
        self._imu.set_connection(conn)

    def start_imu_handler(self) -> bool:
        """Start background IMU reading thread.

        Returns:
            True if started successfully
        """
        return self._imu.start()

    def stop_imu_handler(self) -> None:
        """Stop IMU reading thread."""
        self._imu.stop()

    def process_image(
        self, frame: np.ndarray, timestamp_us: int
    ) -> Optional[VIOState]:
        """Process camera image and update state estimate.

        Args:
            frame: BGR or grayscale camera image
            timestamp_us: Image timestamp in microseconds

        Returns:
            Updated VIO state, or None if not initialized
        """
        if not self._initialized:
            logger.warning("VIO estimator not initialized")
            return None

        with self._lock:
            # Get IMU samples since last VO update
            imu_samples = self._imu.get_samples_between(
                self._last_vo_timestamp_us, timestamp_us
            )
            self._imu_samples_since_vo = len(imu_samples)

            # EKF prediction with IMU
            if imu_samples:
                self._ekf.predict(imu_samples)

            # Visual odometry
            timestamp_s = timestamp_us / 1e6
            motion = self._vo.process_frame(frame, timestamp_s)

            self._vo_attempts += 1
            if motion is not None and motion.valid:
                self._vo_successes += 1

                # Convert VO motion to NED frame
                delta_position, delta_rotation = self._transform_vo_to_ned(motion)

                # EKF update with VO
                self._ekf.update_vo(delta_position, delta_rotation)

                # Accumulate for scale correction
                self._accumulated_vo_distance += np.linalg.norm(delta_position)

            # Update VO success rate
            if self._vo_attempts >= 10:
                self._vo_success_rate = self._vo_successes / self._vo_attempts
                self._vo_attempts = 0
                self._vo_successes = 0

            self._last_vo_timestamp_us = timestamp_us

            return self._get_state()

    def update_with_gps(
        self,
        gps_position: np.ndarray,
        gps_velocity: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ) -> Optional[VIOState]:
        """Update with GPS measurement for hybrid mode.

        Args:
            gps_position: GPS position [north, east, down] in meters
            gps_velocity: GPS velocity [vn, ve, vd] in m/s (optional)
            timestamp: GPS timestamp (optional, uses current time if None)

        Returns:
            Updated VIO state
        """
        if not self._initialized:
            return None

        if timestamp is None:
            timestamp = time.time()

        with self._lock:
            # EKF update with GPS
            self._ekf.update_gps(gps_position, gps_velocity)

            # Scale correction
            if self._last_gps_position is not None:
                gps_distance = np.linalg.norm(gps_position - self._last_gps_position)
                self._accumulated_gps_distance += gps_distance

                # Correct scale when we have enough data
                if (
                    self._accumulated_vo_distance > 5.0
                    and self._accumulated_gps_distance > 5.0
                ):
                    self._vo.correct_scale_with_gps(
                        self._accumulated_vo_distance, self._accumulated_gps_distance
                    )
                    self._accumulated_vo_distance = 0.0
                    self._accumulated_gps_distance = 0.0

            self._last_gps_position = gps_position.copy()
            self._last_gps_timestamp = timestamp

            return self._get_state()

    def add_imu_sample(self, sample: IMUSample) -> None:
        """Manually add IMU sample (for external IMU sources).

        Args:
            sample: IMU sample to add
        """
        self._imu.add_sample(sample)

    def get_state(self) -> Optional[VIOState]:
        """Get current state estimate.

        Returns:
            Current VIO state, or None if not initialized
        """
        with self._lock:
            return self._get_state()

    def get_position(self) -> Tuple[Optional[np.ndarray], NavigationMode]:
        """Get current position and navigation mode.

        Returns:
            Tuple of (position_ned, mode)
        """
        state = self.get_state()
        if state is None:
            return None, NavigationMode.DEAD_RECKONING

        return state.position.copy(), state.mode

    def _get_state(self) -> Optional[VIOState]:
        """Internal method to get state (must hold lock)."""
        if not self._initialized or self._ekf.state is None:
            return None

        ekf_state = self._ekf.state

        # Determine navigation mode
        mode = self._determine_mode()

        return VIOState(
            position=ekf_state.position.copy(),
            velocity=ekf_state.velocity.copy(),
            quaternion=ekf_state.quaternion.copy(),
            position_std=self._ekf.position_std,
            velocity_std=self._ekf.velocity_std,
            mode=mode,
            timestamp_us=ekf_state.timestamp_us,
            vo_quality=self._vo_success_rate,
            scale_confidence=self._vo.scale_confidence,
            imu_sample_count=self._imu_samples_since_vo,
        )

    def _determine_mode(self) -> NavigationMode:
        """Determine current navigation mode."""
        if self.config.mode == NavigationMode.GPS:
            return NavigationMode.GPS
        elif self.config.mode == NavigationMode.VIO:
            return NavigationMode.VIO

        # Hybrid mode - check GPS freshness
        time_since_gps = time.time() - self._last_gps_timestamp
        if time_since_gps < self.config.gps_timeout:
            return NavigationMode.HYBRID
        elif self._vo.is_healthy:
            return NavigationMode.VIO
        else:
            return NavigationMode.DEAD_RECKONING

    def _transform_vo_to_ned(
        self, motion: MotionEstimate
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Transform VO motion from camera frame to NED frame.

        Args:
            motion: VO motion estimate (camera frame)

        Returns:
            Tuple of (delta_position_ned, delta_rotation_ned)
        """
        # Get current orientation
        if self._ekf.state is None:
            return np.zeros(3), np.zeros(3)

        R_body_to_ned = self._ekf.state.rotation_matrix

        # Camera to body frame transform (camera points forward-down)
        # Adjust based on actual camera mounting
        R_cam_to_body = np.array(
            [
                [0, 0, 1],   # Camera Z -> Body X (forward)
                [-1, 0, 0],  # Camera X -> Body -Y (left)
                [0, -1, 0],  # Camera Y -> Body -Z (up)
            ],
            dtype=np.float64,
        )

        # Transform translation
        t_body = R_cam_to_body @ motion.t
        delta_position = R_body_to_ned @ t_body

        # Transform rotation
        R_delta_cam = motion.R
        R_delta_body = R_cam_to_body @ R_delta_cam @ R_cam_to_body.T

        # Extract Euler angles from rotation matrix
        # Using small angle approximation for incremental rotation
        delta_roll = R_delta_body[2, 1]
        delta_pitch = -R_delta_body[2, 0]
        delta_yaw = R_delta_body[1, 0]

        delta_rotation = np.array([delta_roll, delta_pitch, delta_yaw])

        return delta_position, delta_rotation

    def reset(self) -> None:
        """Reset estimator to uninitialized state."""
        with self._lock:
            self._ekf.reset()
            self._initialized = False
            self._last_vo_timestamp_us = 0
            self._last_gps_timestamp = 0.0
            self._accumulated_vo_distance = 0.0
            self._accumulated_gps_distance = 0.0
            self._last_gps_position = None
            logger.info(f"VIO estimator reset for drone {self.drone_id}")

    @property
    def is_initialized(self) -> bool:
        """Whether estimator is initialized."""
        return self._initialized

    @property
    def is_healthy(self) -> bool:
        """Whether estimator is producing valid estimates."""
        return self._initialized and self._vo.is_healthy

    @property
    def vo_quality(self) -> float:
        """Visual odometry quality (0-1)."""
        return self._vo_success_rate

    @property
    def scale(self) -> float:
        """Current metric scale estimate."""
        return self._vo.scale

    @property
    def scale_confidence(self) -> float:
        """Confidence in scale estimate (0-1)."""
        return self._vo.scale_confidence

    @property
    def imu_rate(self) -> float:
        """Current IMU sample rate (Hz)."""
        return self._imu.sample_rate
