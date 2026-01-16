"""Extended Kalman Filter for Visual-Inertial Odometry."""

import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
from scipy.linalg import block_diag

from .config import EKFConfig
from .imu_handler import IMUSample
from .utils import (
    quaternion_from_rotation_vector,
    quaternion_multiply,
    rotation_matrix_from_quaternion,
    skew_symmetric,
    euler_from_quaternion,
)

logger = logging.getLogger(__name__)


@dataclass
class EKFState:
    """EKF state vector components."""

    # Position in NED frame (3)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Velocity in NED frame (3)
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Orientation as quaternion [w, x, y, z] (4)
    quaternion: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0])
    )

    # Accelerometer bias (3)
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Gyroscope bias (3)
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Error-state covariance matrix (15x15)
    # Order: [position(3), velocity(3), attitude(3), accel_bias(3), gyro_bias(3)]
    P: np.ndarray = field(default_factory=lambda: np.eye(15))

    # Timestamp in microseconds
    timestamp_us: int = 0

    def __post_init__(self):
        """Ensure arrays are numpy."""
        self.position = np.asarray(self.position, dtype=np.float64)
        self.velocity = np.asarray(self.velocity, dtype=np.float64)
        self.quaternion = np.asarray(self.quaternion, dtype=np.float64)
        self.accel_bias = np.asarray(self.accel_bias, dtype=np.float64)
        self.gyro_bias = np.asarray(self.gyro_bias, dtype=np.float64)
        self.P = np.asarray(self.P, dtype=np.float64)

        # Normalize quaternion
        self.quaternion = self.quaternion / np.linalg.norm(self.quaternion)

    @property
    def attitude_euler(self) -> Tuple[float, float, float]:
        """Get attitude as Euler angles (roll, pitch, yaw)."""
        return euler_from_quaternion(self.quaternion)

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Get rotation matrix from body to NED."""
        return rotation_matrix_from_quaternion(self.quaternion)

    def copy(self) -> "EKFState":
        """Create a deep copy of the state."""
        return EKFState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            quaternion=self.quaternion.copy(),
            accel_bias=self.accel_bias.copy(),
            gyro_bias=self.gyro_bias.copy(),
            P=self.P.copy(),
            timestamp_us=self.timestamp_us,
        )


class NavigationEKF:
    """Extended Kalman Filter for VIO fusion.

    Uses error-state formulation with 15-dimensional error state:
    - Position error (3)
    - Velocity error (3)
    - Attitude error (3) - rotation vector
    - Accelerometer bias error (3)
    - Gyroscope bias error (3)
    """

    # State indices
    IDX_POS = slice(0, 3)
    IDX_VEL = slice(3, 6)
    IDX_ATT = slice(6, 9)
    IDX_BA = slice(9, 12)
    IDX_BG = slice(12, 15)

    # Gravity in NED frame (m/s^2)
    GRAVITY = np.array([0.0, 0.0, 9.81])

    def __init__(self, config: EKFConfig):
        """Initialize EKF.

        Args:
            config: EKF configuration
        """
        self.config = config
        self._state: Optional[EKFState] = None
        self._initialized = False

    def initialize(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        quaternion: np.ndarray,
        timestamp_us: int,
        accel_bias: Optional[np.ndarray] = None,
        gyro_bias: Optional[np.ndarray] = None,
    ) -> None:
        """Initialize filter state.

        Args:
            position: Initial position [north, east, down] in meters
            velocity: Initial velocity [vn, ve, vd] in m/s
            quaternion: Initial orientation [w, x, y, z]
            timestamp_us: Initial timestamp in microseconds
            accel_bias: Initial accelerometer bias (default zeros)
            gyro_bias: Initial gyroscope bias (default zeros)
        """
        # Initial covariance
        P0 = np.diag(
            [
                # Position
                self.config.initial_position_var,
                self.config.initial_position_var,
                self.config.initial_position_var,
                # Velocity
                self.config.initial_velocity_var,
                self.config.initial_velocity_var,
                self.config.initial_velocity_var,
                # Attitude
                self.config.initial_attitude_var,
                self.config.initial_attitude_var,
                self.config.initial_attitude_var,
                # Accelerometer bias
                self.config.initial_accel_bias_var,
                self.config.initial_accel_bias_var,
                self.config.initial_accel_bias_var,
                # Gyroscope bias
                self.config.initial_gyro_bias_var,
                self.config.initial_gyro_bias_var,
                self.config.initial_gyro_bias_var,
            ]
        )

        self._state = EKFState(
            position=np.asarray(position),
            velocity=np.asarray(velocity),
            quaternion=np.asarray(quaternion),
            accel_bias=np.asarray(accel_bias) if accel_bias is not None else np.zeros(3),
            gyro_bias=np.asarray(gyro_bias) if gyro_bias is not None else np.zeros(3),
            P=P0,
            timestamp_us=timestamp_us,
        )

        self._initialized = True
        logger.info(f"EKF initialized at position: {position}")

    def predict(self, imu_samples: list[IMUSample]) -> Optional[EKFState]:
        """Prediction step using IMU integration.

        Integrates accelerometer and gyroscope measurements between
        camera frames to propagate state forward.

        Args:
            imu_samples: List of IMU samples to integrate (sorted by time)

        Returns:
            Updated state after prediction, or None if not initialized
        """
        if not self._initialized or self._state is None:
            logger.warning("EKF not initialized")
            return None

        if not imu_samples:
            return self._state

        for sample in imu_samples:
            # Skip if sample is older than current state
            if sample.timestamp_us <= self._state.timestamp_us:
                continue

            dt = (sample.timestamp_us - self._state.timestamp_us) / 1e6

            # Skip if time gap is too large or invalid
            if dt <= 0 or dt > 0.5:
                logger.warning(f"Skipping IMU sample with dt={dt:.3f}s")
                self._state.timestamp_us = sample.timestamp_us
                continue

            # Remove estimated biases
            accel = sample.accel - self._state.accel_bias
            gyro = sample.gyro - self._state.gyro_bias

            # Current rotation matrix
            R = self._state.rotation_matrix

            # Propagate orientation using gyroscope
            omega = gyro * dt
            dq = quaternion_from_rotation_vector(omega)
            self._state.quaternion = quaternion_multiply(self._state.quaternion, dq)
            self._state.quaternion /= np.linalg.norm(self._state.quaternion)

            # Rotate acceleration to NED frame and remove gravity
            accel_ned = R @ accel - self.GRAVITY

            # Propagate velocity and position
            self._state.velocity = self._state.velocity + accel_ned * dt
            self._state.position = (
                self._state.position
                + self._state.velocity * dt
                + 0.5 * accel_ned * dt * dt
            )

            # Propagate covariance
            F = self._compute_state_transition(accel, dt)
            Q = self._compute_process_noise(dt)
            self._state.P = F @ self._state.P @ F.T + Q

            # Enforce covariance bounds for numerical stability
            self._enforce_covariance_bounds()

            self._state.timestamp_us = sample.timestamp_us

        return self._state

    def update_vo(
        self,
        delta_position: np.ndarray,
        delta_rotation: np.ndarray,
        vo_covariance: Optional[np.ndarray] = None,
    ) -> Optional[EKFState]:
        """Update step with visual odometry measurement.

        Args:
            delta_position: Relative position change [dx, dy, dz] in NED
            delta_rotation: Relative rotation [droll, dpitch, dyaw] in radians
            vo_covariance: 6x6 measurement covariance (position + rotation)

        Returns:
            Updated state after measurement, or None if not initialized
        """
        if not self._initialized or self._state is None:
            return None

        # Default covariance if not provided
        if vo_covariance is None:
            vo_covariance = np.diag(
                [
                    self.config.vo_position_noise ** 2,
                    self.config.vo_position_noise ** 2,
                    self.config.vo_position_noise ** 2,
                    self.config.vo_rotation_noise ** 2,
                    self.config.vo_rotation_noise ** 2,
                    self.config.vo_rotation_noise ** 2,
                ]
            )

        # Measurement model: H maps error state to VO measurement
        # VO measures relative pose change, which relates to position and attitude errors
        H = np.zeros((6, 15))
        H[0:3, self.IDX_POS] = np.eye(3)  # Position
        H[3:6, self.IDX_ATT] = np.eye(3)  # Attitude

        # Innovation (measurement - prediction)
        # For VO, we expect the relative change to match our prediction
        # The "predicted" measurement is zero (no accumulated error)
        z = np.concatenate([delta_position, delta_rotation])
        z_pred = np.zeros(6)  # Error state should be zero
        y = z - z_pred

        # Outlier rejection using Mahalanobis distance
        S = H @ self._state.P @ H.T + vo_covariance
        try:
            S_inv = np.linalg.inv(S)
            mahal_dist = y.T @ S_inv @ y
            if mahal_dist > self.config.mahalanobis_threshold * 6:  # 6 DOF
                logger.warning(f"VO update rejected: Mahalanobis distance {mahal_dist:.2f}")
                return self._state
        except np.linalg.LinAlgError:
            logger.error("Singular innovation covariance")
            return self._state

        # Kalman gain
        K = self._state.P @ H.T @ S_inv

        # State correction
        dx = K @ y
        self._apply_error_state_correction(dx)

        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(15) - K @ H
        self._state.P = (
            I_KH @ self._state.P @ I_KH.T + K @ vo_covariance @ K.T
        )

        self._enforce_covariance_bounds()

        return self._state

    def update_gps(
        self,
        gps_position: np.ndarray,
        gps_velocity: Optional[np.ndarray] = None,
        gps_covariance: Optional[np.ndarray] = None,
    ) -> Optional[EKFState]:
        """Update step with GPS measurement (for hybrid mode).

        Args:
            gps_position: GPS position [north, east, down] in meters
            gps_velocity: GPS velocity [vn, ve, vd] in m/s (optional)
            gps_covariance: Measurement covariance (optional)

        Returns:
            Updated state after measurement, or None if not initialized
        """
        if not self._initialized or self._state is None:
            return None

        # Determine measurement dimension
        if gps_velocity is not None:
            z = np.concatenate([gps_position, gps_velocity])
            n_meas = 6

            if gps_covariance is None:
                gps_covariance = np.diag(
                    [
                        self.config.gps_position_noise ** 2,
                        self.config.gps_position_noise ** 2,
                        self.config.gps_position_noise ** 2,
                        self.config.gps_velocity_noise ** 2,
                        self.config.gps_velocity_noise ** 2,
                        self.config.gps_velocity_noise ** 2,
                    ]
                )

            # Measurement model
            H = np.zeros((6, 15))
            H[0:3, self.IDX_POS] = np.eye(3)
            H[3:6, self.IDX_VEL] = np.eye(3)

            z_pred = np.concatenate([self._state.position, self._state.velocity])

        else:
            z = gps_position
            n_meas = 3

            if gps_covariance is None:
                gps_covariance = np.diag(
                    [
                        self.config.gps_position_noise ** 2,
                        self.config.gps_position_noise ** 2,
                        self.config.gps_position_noise ** 2,
                    ]
                )

            H = np.zeros((3, 15))
            H[0:3, self.IDX_POS] = np.eye(3)

            z_pred = self._state.position

        # Innovation
        y = z - z_pred

        # Innovation covariance
        S = H @ self._state.P @ H.T + gps_covariance

        # Outlier rejection
        try:
            S_inv = np.linalg.inv(S)
            mahal_dist = y.T @ S_inv @ y
            if mahal_dist > self.config.mahalanobis_threshold * n_meas:
                logger.warning(f"GPS update rejected: Mahalanobis distance {mahal_dist:.2f}")
                return self._state
        except np.linalg.LinAlgError:
            logger.error("Singular GPS innovation covariance")
            return self._state

        # Kalman gain
        K = self._state.P @ H.T @ S_inv

        # State correction
        dx = K @ y
        self._apply_error_state_correction(dx)

        # Covariance update
        I_KH = np.eye(15) - K @ H
        self._state.P = I_KH @ self._state.P @ I_KH.T + K @ gps_covariance @ K.T

        self._enforce_covariance_bounds()

        logger.debug(f"GPS update: position error {np.linalg.norm(y[:3]):.3f}m")

        return self._state

    def _compute_state_transition(self, accel: np.ndarray, dt: float) -> np.ndarray:
        """Compute state transition matrix F for error-state propagation.

        Args:
            accel: Bias-corrected acceleration in body frame
            dt: Time step in seconds

        Returns:
            15x15 state transition matrix
        """
        R = self._state.rotation_matrix

        F = np.eye(15)

        # Position depends on velocity
        F[self.IDX_POS, self.IDX_VEL] = np.eye(3) * dt

        # Velocity depends on attitude error (rotation of gravity)
        F[self.IDX_VEL, self.IDX_ATT] = -R @ skew_symmetric(accel) * dt

        # Velocity depends on accelerometer bias
        F[self.IDX_VEL, self.IDX_BA] = -R * dt

        # Attitude depends on gyroscope bias
        F[self.IDX_ATT, self.IDX_BG] = -np.eye(3) * dt

        return F

    def _compute_process_noise(self, dt: float) -> np.ndarray:
        """Compute process noise covariance Q.

        Args:
            dt: Time step in seconds

        Returns:
            15x15 process noise covariance matrix
        """
        # Discrete-time process noise
        Q = np.diag(
            [
                # Position
                self.config.position_process_noise ** 2 * dt,
                self.config.position_process_noise ** 2 * dt,
                self.config.position_process_noise ** 2 * dt,
                # Velocity
                self.config.velocity_process_noise ** 2 * dt,
                self.config.velocity_process_noise ** 2 * dt,
                self.config.velocity_process_noise ** 2 * dt,
                # Attitude
                self.config.attitude_process_noise ** 2 * dt,
                self.config.attitude_process_noise ** 2 * dt,
                self.config.attitude_process_noise ** 2 * dt,
                # Accelerometer bias
                self.config.accel_bias_process_noise ** 2 * dt,
                self.config.accel_bias_process_noise ** 2 * dt,
                self.config.accel_bias_process_noise ** 2 * dt,
                # Gyroscope bias
                self.config.gyro_bias_process_noise ** 2 * dt,
                self.config.gyro_bias_process_noise ** 2 * dt,
                self.config.gyro_bias_process_noise ** 2 * dt,
            ]
        )

        return Q

    def _apply_error_state_correction(self, dx: np.ndarray) -> None:
        """Apply error state correction to nominal state.

        Args:
            dx: 15-element error state correction vector
        """
        # Position correction
        self._state.position = self._state.position + dx[self.IDX_POS]

        # Velocity correction
        self._state.velocity = self._state.velocity + dx[self.IDX_VEL]

        # Attitude correction (rotation vector to quaternion)
        dtheta = dx[self.IDX_ATT]
        dq = quaternion_from_rotation_vector(dtheta)
        self._state.quaternion = quaternion_multiply(self._state.quaternion, dq)
        self._state.quaternion /= np.linalg.norm(self._state.quaternion)

        # Bias corrections
        self._state.accel_bias = self._state.accel_bias + dx[self.IDX_BA]
        self._state.gyro_bias = self._state.gyro_bias + dx[self.IDX_BG]

    def _enforce_covariance_bounds(self) -> None:
        """Enforce min/max bounds on covariance for numerical stability."""
        # Ensure symmetry
        self._state.P = 0.5 * (self._state.P + self._state.P.T)

        # Clip diagonal elements
        diag = np.diag(self._state.P)
        diag = np.clip(diag, self.config.min_covariance, self.config.max_covariance)
        np.fill_diagonal(self._state.P, diag)

    @property
    def state(self) -> Optional[EKFState]:
        """Get current state."""
        return self._state

    @property
    def is_initialized(self) -> bool:
        """Whether filter is initialized."""
        return self._initialized

    @property
    def position(self) -> Optional[np.ndarray]:
        """Get current position estimate."""
        return self._state.position if self._state else None

    @property
    def velocity(self) -> Optional[np.ndarray]:
        """Get current velocity estimate."""
        return self._state.velocity if self._state else None

    @property
    def quaternion(self) -> Optional[np.ndarray]:
        """Get current orientation quaternion."""
        return self._state.quaternion if self._state else None

    @property
    def position_std(self) -> Optional[np.ndarray]:
        """Get position standard deviation."""
        if self._state is None:
            return None
        return np.sqrt(np.diag(self._state.P)[self.IDX_POS])

    @property
    def velocity_std(self) -> Optional[np.ndarray]:
        """Get velocity standard deviation."""
        if self._state is None:
            return None
        return np.sqrt(np.diag(self._state.P)[self.IDX_VEL])

    def reset(self) -> None:
        """Reset filter to uninitialized state."""
        self._state = None
        self._initialized = False
        logger.info("EKF reset")
