"""Unit tests for GPS-denied navigation module."""

import numpy as np
import pytest
import time

from swarm.navigation import (
    NavigationConfig,
    NavigationMode,
    VOConfig,
    IMUConfig,
    EKFConfig,
    CameraConfig,
    NavigationEKF,
    EKFState,
    IMUHandler,
    IMUBuffer,
    IMUSample,
    VisualOdometry,
    ORBMatcher,
    ScaleEstimator,
    VIOEstimator,
)
from swarm.navigation.utils import (
    rotation_matrix_from_euler,
    euler_from_rotation_matrix,
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_multiply,
    body_to_ned,
    ned_to_body,
    skew_symmetric,
)


class TestCoordinateTransforms:
    """Test coordinate transformation utilities."""

    def test_euler_to_rotation_and_back(self):
        """Test Euler to rotation matrix and back."""
        roll, pitch, yaw = 0.1, 0.2, 0.3
        R = rotation_matrix_from_euler(roll, pitch, yaw)
        roll2, pitch2, yaw2 = euler_from_rotation_matrix(R)

        assert abs(roll - roll2) < 1e-10
        assert abs(pitch - pitch2) < 1e-10
        assert abs(yaw - yaw2) < 1e-10

    def test_quaternion_euler_conversion(self):
        """Test quaternion to Euler conversion."""
        roll, pitch, yaw = 0.1, 0.2, 0.3
        q = quaternion_from_euler(roll, pitch, yaw)

        # Quaternion should be unit
        assert abs(np.linalg.norm(q) - 1.0) < 1e-10

        roll2, pitch2, yaw2 = euler_from_quaternion(q)
        assert abs(roll - roll2) < 1e-10
        assert abs(pitch - pitch2) < 1e-10
        assert abs(yaw - yaw2) < 1e-10

    def test_quaternion_multiply(self):
        """Test quaternion multiplication."""
        q1 = quaternion_from_euler(0.1, 0.0, 0.0)
        q2 = quaternion_from_euler(0.0, 0.1, 0.0)
        q3 = quaternion_multiply(q1, q2)

        # Result should be unit quaternion
        assert abs(np.linalg.norm(q3) - 1.0) < 1e-10

    def test_body_to_ned_identity(self):
        """Test body to NED with identity rotation."""
        v = np.array([1.0, 0.0, 0.0])
        attitude = np.array([0.0, 0.0, 0.0])
        v_ned = body_to_ned(v, attitude)

        np.testing.assert_array_almost_equal(v, v_ned)

    def test_ned_to_body_and_back(self):
        """Test NED to body and back."""
        v = np.array([1.0, 2.0, 3.0])
        attitude = np.array([0.1, 0.2, 0.3])

        v_body = ned_to_body(v, attitude)
        v_ned = body_to_ned(v_body, attitude)

        np.testing.assert_array_almost_equal(v, v_ned)

    def test_skew_symmetric(self):
        """Test skew symmetric matrix."""
        v = np.array([1.0, 2.0, 3.0])
        S = skew_symmetric(v)

        # Check antisymmetry
        np.testing.assert_array_almost_equal(S, -S.T)

        # Check cross product property: S @ u = v x u
        u = np.array([4.0, 5.0, 6.0])
        np.testing.assert_array_almost_equal(S @ u, np.cross(v, u))


class TestIMUBuffer:
    """Test IMU buffer functionality."""

    def test_add_and_get_latest(self):
        """Test adding samples and getting latest."""
        buffer = IMUBuffer(max_size=10)

        for i in range(5):
            sample = IMUSample(
                timestamp_us=i * 1000,
                accel=np.array([0.0, 0.0, 9.81]),
                gyro=np.zeros(3),
            )
            buffer.add(sample)

        latest = buffer.get_latest(1)
        assert len(latest) == 1
        assert latest[0].timestamp_us == 4000

    def test_get_samples_between(self):
        """Test getting samples in time range."""
        buffer = IMUBuffer(max_size=100)

        for i in range(10):
            sample = IMUSample(
                timestamp_us=i * 1000,
                accel=np.array([0.0, 0.0, 9.81]),
                gyro=np.zeros(3),
            )
            buffer.add(sample)

        samples = buffer.get_samples_between(2000, 7000)
        assert len(samples) == 6  # Inclusive: 2, 3, 4, 5, 6, 7
        assert samples[0].timestamp_us == 2000
        assert samples[-1].timestamp_us == 7000

    def test_buffer_overflow(self):
        """Test buffer correctly handles overflow."""
        buffer = IMUBuffer(max_size=5)

        for i in range(10):
            sample = IMUSample(
                timestamp_us=i * 1000,
                accel=np.zeros(3),
                gyro=np.zeros(3),
            )
            buffer.add(sample)

        assert buffer.size == 5
        assert buffer.oldest_timestamp_us == 5000
        assert buffer.latest_timestamp_us == 9000


class TestIMUSample:
    """Test IMU sample dataclass."""

    def test_timestamp_conversion(self):
        """Test timestamp conversion."""
        sample = IMUSample(
            timestamp_us=1_500_000,
            accel=np.array([0.0, 0.0, 9.81]),
            gyro=np.zeros(3),
        )
        assert abs(sample.timestamp_s - 1.5) < 1e-10


class TestEKFState:
    """Test EKF state dataclass."""

    def test_default_initialization(self):
        """Test default state initialization."""
        state = EKFState()

        np.testing.assert_array_equal(state.position, np.zeros(3))
        np.testing.assert_array_equal(state.velocity, np.zeros(3))
        np.testing.assert_array_equal(state.quaternion, [1, 0, 0, 0])
        assert state.P.shape == (15, 15)

    def test_copy(self):
        """Test state copy."""
        state = EKFState(position=np.array([1, 2, 3]))
        state_copy = state.copy()

        # Modify original
        state.position[0] = 100

        # Copy should be unchanged
        assert state_copy.position[0] == 1


class TestNavigationEKF:
    """Test Extended Kalman Filter."""

    def test_initialization(self):
        """Test EKF initialization."""
        config = EKFConfig()
        ekf = NavigationEKF(config)

        assert not ekf.is_initialized

        ekf.initialize(
            position=np.array([1, 2, 3]),
            velocity=np.zeros(3),
            quaternion=np.array([1, 0, 0, 0]),
            timestamp_us=0,
        )

        assert ekf.is_initialized
        np.testing.assert_array_almost_equal(ekf.position, [1, 2, 3])

    def test_predict_constant_velocity(self):
        """Test prediction with constant velocity motion."""
        config = EKFConfig()
        ekf = NavigationEKF(config)

        ekf.initialize(
            position=np.zeros(3),
            velocity=np.array([1, 0, 0]),  # 1 m/s north
            quaternion=np.array([1, 0, 0, 0]),
            timestamp_us=0,
        )

        # Create IMU samples (level flight, no rotation)
        samples = []
        for i in range(100):  # 100ms of data
            sample = IMUSample(
                timestamp_us=i * 1000,  # 1ms intervals
                accel=np.array([0, 0, -9.81]),  # Gravity in body frame (level)
                gyro=np.zeros(3),
            )
            samples.append(sample)

        state = ekf.predict(samples)

        # Position should have moved ~0.1m north
        assert state.position[0] > 0  # Moved north

    def test_reset(self):
        """Test filter reset."""
        config = EKFConfig()
        ekf = NavigationEKF(config)

        ekf.initialize(
            position=np.array([1, 2, 3]),
            velocity=np.zeros(3),
            quaternion=np.array([1, 0, 0, 0]),
            timestamp_us=0,
        )

        assert ekf.is_initialized

        ekf.reset()

        assert not ekf.is_initialized
        assert ekf.state is None


class TestScaleEstimator:
    """Test monocular scale estimator."""

    def test_initialization(self):
        """Test scale initialization."""
        config = VOConfig()
        estimator = ScaleEstimator(config)

        estimator.initialize(altitude=10.0)

        assert estimator.scale == 1.0
        assert estimator.scale_confidence < 0.5  # Low initial confidence

    def test_scale_from_altitude(self):
        """Test scale update from altitude."""
        config = VOConfig()
        estimator = ScaleEstimator(config)
        estimator.initialize(altitude=10.0)

        # Simulated VO says we moved 1 unit down
        # Barometer says we moved 2 meters down
        # So scale should be 2
        scale = estimator.update_from_altitude(
            vo_delta_z=1.0,
            baro_delta_z=2.0,
        )

        assert scale > 1.0  # Scale increased


class TestNavigationConfig:
    """Test configuration classes."""

    def test_for_simulation(self):
        """Test simulation config factory."""
        config = NavigationConfig.for_simulation()

        assert config.mode == NavigationMode.HYBRID
        assert config.vo.use_gpu is True

    def test_for_gps_denied(self):
        """Test GPS-denied config factory."""
        config = NavigationConfig.for_gps_denied()

        assert config.mode == NavigationMode.VIO

    def test_camera_intrinsics(self):
        """Test camera intrinsic matrix."""
        camera = CameraConfig()

        K = camera.K
        assert K.shape == (3, 3)
        assert K[0, 0] == camera.fx
        assert K[1, 1] == camera.fy
        assert K[0, 2] == camera.cx
        assert K[1, 2] == camera.cy


class TestORBMatcher:
    """Test ORB feature matching (requires OpenCV)."""

    @pytest.fixture
    def matcher(self):
        """Create ORB matcher."""
        vo_config = VOConfig(use_gpu=False)  # Force CPU for tests
        camera_config = CameraConfig()
        return ORBMatcher(vo_config, camera_config)

    def test_initialization(self, matcher):
        """Test matcher initialization."""
        success = matcher.initialize()
        assert success

    def test_process_synthetic_frame(self, matcher):
        """Test processing a synthetic frame."""
        matcher.initialize()

        # Create synthetic frame with features
        frame = np.random.randint(0, 255, (480, 640), dtype=np.uint8)

        # Add some bright spots for features
        for i in range(50):
            x = np.random.randint(50, 590)
            y = np.random.randint(50, 430)
            frame[y-5:y+5, x-5:x+5] = 255

        result = matcher.process_frame(frame, timestamp=0.0)

        # Should extract some features
        assert result is not None
        assert len(result.keypoints) > 0


class TestVIOEstimator:
    """Test VIO estimator integration."""

    def test_initialization(self):
        """Test VIO estimator initialization."""
        config = NavigationConfig.for_simulation()
        config.vo.use_gpu = False  # Force CPU for tests

        vio = VIOEstimator(config, drone_id=0)

        success = vio.initialize(
            position=np.zeros(3),
            velocity=np.zeros(3),
            attitude=np.zeros(3),
            altitude=10.0,
            timestamp_us=0,
        )

        assert success
        assert vio.is_initialized

    def test_get_state(self):
        """Test getting VIO state."""
        config = NavigationConfig.for_simulation()
        config.vo.use_gpu = False

        vio = VIOEstimator(config, drone_id=0)
        vio.initialize(
            position=np.array([1, 2, 3]),
            velocity=np.zeros(3),
            attitude=np.zeros(3),
            altitude=10.0,
            timestamp_us=0,
        )

        state = vio.get_state()

        assert state is not None
        np.testing.assert_array_almost_equal(state.position, [1, 2, 3])


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
