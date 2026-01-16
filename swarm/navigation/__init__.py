"""Navigation modules for GPS-denied navigation with visual-inertial odometry.

This package provides:
- Visual odometry using ORB features with optical flow fallback
- IMU data handling and integration
- Extended Kalman Filter for sensor fusion
- Hybrid GPS/VIO position source

Example usage:
    from swarm.navigation import (
        NavigationConfig,
        NavigationMode,
        VIOEstimator,
        PositionSource,
    )

    # Create configuration
    config = NavigationConfig.for_simulation()

    # Create VIO estimator
    vio = VIOEstimator(config, drone_id=0)
    vio.set_mavlink_connection(mavlink_conn)
    vio.initialize(
        position=np.array([0, 0, -10]),
        velocity=np.zeros(3),
        attitude=np.array([0, 0, 0]),
        altitude=10.0,
        timestamp_us=int(time.time() * 1e6),
    )
    vio.start_imu_handler()

    # Process camera frames
    state = vio.process_image(frame, timestamp_us)
    if state:
        print(f"Position: {state.position}, Mode: {state.mode}")
"""

from .config import (
    NavigationConfig,
    NavigationMode,
    VOConfig,
    IMUConfig,
    EKFConfig,
    CameraConfig,
)

from .ekf import NavigationEKF, EKFState

from .imu_handler import (
    IMUHandler,
    IMUBuffer,
    IMUSample,
    integrate_imu,
)

from .visual_odometry import (
    VisualOdometry,
    ORBMatcher,
    OpticalFlowFallback,
    ScaleEstimator,
    MotionEstimate,
    FrameResult,
    MatchResult,
)

from .vio_estimator import VIOEstimator, VIOState

from .position_source import (
    PositionSource,
    GPSData,
    GPSDenialSimulator,
)

__all__ = [
    # Configuration
    "NavigationConfig",
    "NavigationMode",
    "VOConfig",
    "IMUConfig",
    "EKFConfig",
    "CameraConfig",
    # EKF
    "NavigationEKF",
    "EKFState",
    # IMU
    "IMUHandler",
    "IMUBuffer",
    "IMUSample",
    "integrate_imu",
    # Visual Odometry
    "VisualOdometry",
    "ORBMatcher",
    "OpticalFlowFallback",
    "ScaleEstimator",
    "MotionEstimate",
    "FrameResult",
    "MatchResult",
    # VIO Estimator
    "VIOEstimator",
    "VIOState",
    # Position Source
    "PositionSource",
    "GPSData",
    "GPSDenialSimulator",
]
