"""Configuration dataclasses for GPS-denied navigation."""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

import numpy as np


class NavigationMode(IntEnum):
    """Navigation mode enumeration."""

    GPS = 0  # GPS-based navigation (ArduPilot EKF)
    VIO = 1  # Visual-Inertial Odometry only
    HYBRID = 2  # GPS when available, VIO when denied
    DEAD_RECKONING = 3  # IMU-only (last resort)


@dataclass
class VOConfig:
    """Visual odometry configuration."""

    # ORB feature parameters
    max_features: int = 1000  # Maximum ORB features per frame
    match_ratio: float = 0.7  # Lowe's ratio test threshold
    min_matches: int = 20  # Minimum matches for valid VO
    ransac_threshold: float = 1.0  # RANSAC reprojection threshold (pixels)
    ransac_confidence: float = 0.999  # RANSAC confidence level

    # GPU acceleration
    use_gpu: bool = True  # Use GPU if available (CUDA)

    # Frame processing
    frame_skip: int = 1  # Process every Nth frame (1 = all frames)
    max_frame_delay: float = 0.2  # Max delay before skipping frame (seconds)

    # Optical flow fallback parameters
    flow_win_size: int = 21  # Lucas-Kanade window size
    flow_max_level: int = 3  # Image pyramid levels
    flow_min_displacement: float = 0.5  # Minimum pixel displacement
    flow_max_corners: int = 200  # Shi-Tomasi corners for flow

    # Scale estimation
    initial_scale: float = 1.0  # Initial metric scale
    scale_filter_alpha: float = 0.1  # EMA filter for scale updates


@dataclass
class IMUConfig:
    """IMU handler configuration."""

    # Buffer settings
    buffer_size: int = 500  # Samples (~2.5s at 200Hz)
    expected_rate_hz: float = 200.0  # Expected IMU rate

    # Noise parameters (from ArduPilot defaults)
    accel_noise: float = 0.35  # m/s^2 (accelerometer noise density)
    gyro_noise: float = 0.05  # rad/s (gyroscope noise density)
    accel_bias_noise: float = 0.001  # m/s^2 (bias random walk)
    gyro_bias_noise: float = 0.0001  # rad/s (bias random walk)

    # Initial bias estimates
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Message types to read
    message_types: tuple = ("RAW_IMU", "SCALED_IMU", "HIGHRES_IMU")


@dataclass
class EKFConfig:
    """Extended Kalman Filter configuration."""

    # Process noise (Q matrix diagonal elements)
    position_process_noise: float = 0.01  # m (position random walk)
    velocity_process_noise: float = 0.1  # m/s (velocity process noise)
    attitude_process_noise: float = 0.001  # rad (attitude process noise)
    accel_bias_process_noise: float = 0.0001  # m/s^2 (accelerometer bias drift)
    gyro_bias_process_noise: float = 0.00001  # rad/s (gyroscope bias drift)

    # Measurement noise (R matrix diagonal elements)
    vo_position_noise: float = 0.5  # m (VO position measurement noise)
    vo_rotation_noise: float = 0.1  # rad (VO rotation measurement noise)
    gps_position_noise: float = 1.0  # m (GPS position noise)
    gps_velocity_noise: float = 0.3  # m/s (GPS velocity noise)

    # Initial state covariance (P0 diagonal elements)
    initial_position_var: float = 1.0  # m^2
    initial_velocity_var: float = 0.5  # (m/s)^2
    initial_attitude_var: float = 0.1  # rad^2
    initial_accel_bias_var: float = 0.1  # (m/s^2)^2
    initial_gyro_bias_var: float = 0.01  # (rad/s)^2

    # Outlier rejection
    mahalanobis_threshold: float = 5.991  # Chi-squared 95% for 2 DOF

    # Numerical stability
    min_covariance: float = 1e-10  # Minimum covariance diagonal
    max_covariance: float = 1e6  # Maximum covariance diagonal


@dataclass
class CameraConfig:
    """Camera intrinsic parameters."""

    # Image dimensions
    width: int = 640
    height: int = 480

    # Focal length (pixels) - approximate for 80 deg FOV
    fx: float = 462.0
    fy: float = 462.0

    # Principal point (image center)
    cx: float = 320.0
    cy: float = 240.0

    # Distortion coefficients (assume undistorted from Gazebo)
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0

    @property
    def K(self) -> np.ndarray:
        """Camera intrinsic matrix (3x3)."""
        return np.array(
            [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64
        )

    @property
    def dist_coeffs(self) -> np.ndarray:
        """Distortion coefficients array."""
        return np.array([self.k1, self.k2, self.p1, self.p2], dtype=np.float64)


@dataclass
class NavigationConfig:
    """Top-level navigation configuration."""

    # Operating mode
    mode: NavigationMode = NavigationMode.HYBRID

    # GPS timeout for hybrid mode
    gps_timeout: float = 3.0  # seconds without GPS before switching to VIO

    # Position source switching hysteresis
    switch_back_delay: float = 2.0  # seconds of good GPS before switching back

    # Submodule configurations
    vo: VOConfig = field(default_factory=VOConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    ekf: EKFConfig = field(default_factory=EKFConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)

    # Logging
    log_level: str = "INFO"
    log_diagnostics: bool = True  # Log EKF diagnostics

    @classmethod
    def for_simulation(cls) -> "NavigationConfig":
        """Create configuration optimized for Gazebo simulation."""
        return cls(
            mode=NavigationMode.HYBRID,
            vo=VOConfig(use_gpu=True, max_features=1000),
            imu=IMUConfig(
                # Simulation has less noise
                accel_noise=0.1,
                gyro_noise=0.01,
            ),
            ekf=EKFConfig(
                # Tighter noise for simulation
                vo_position_noise=0.3,
                gps_position_noise=0.5,
            ),
        )

    @classmethod
    def for_gps_denied(cls) -> "NavigationConfig":
        """Create configuration for GPS-denied operation."""
        return cls(
            mode=NavigationMode.VIO,
            vo=VOConfig(
                max_features=1500,  # More features for robustness
                min_matches=15,  # Lower threshold
            ),
            ekf=EKFConfig(
                # Trust VO more in GPS-denied
                vo_position_noise=0.3,
            ),
        )
