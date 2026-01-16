"""IMU data handling for GPS-denied navigation."""

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from .config import IMUConfig

logger = logging.getLogger(__name__)


@dataclass
class IMUSample:
    """Single IMU measurement."""

    timestamp_us: int  # Microseconds since boot
    accel: np.ndarray  # [ax, ay, az] m/s^2 in body frame
    gyro: np.ndarray  # [gx, gy, gz] rad/s in body frame

    def __post_init__(self):
        """Ensure arrays are numpy."""
        if not isinstance(self.accel, np.ndarray):
            self.accel = np.array(self.accel, dtype=np.float64)
        if not isinstance(self.gyro, np.ndarray):
            self.gyro = np.array(self.gyro, dtype=np.float64)

    @property
    def timestamp_s(self) -> float:
        """Timestamp in seconds."""
        return self.timestamp_us / 1e6


class IMUBuffer:
    """Thread-safe circular buffer for high-rate IMU data."""

    def __init__(self, max_size: int = 500):
        """Initialize buffer.

        Args:
            max_size: Maximum number of samples to store
        """
        self._buffer: deque[IMUSample] = deque(maxlen=max_size)
        self._lock = threading.Lock()
        self._sample_count = 0

    def add(self, sample: IMUSample) -> None:
        """Add new IMU sample.

        Args:
            sample: IMU sample to add
        """
        with self._lock:
            self._buffer.append(sample)
            self._sample_count += 1

    def get_samples_between(
        self, t_start_us: int, t_end_us: int
    ) -> list[IMUSample]:
        """Get all samples between two timestamps.

        Args:
            t_start_us: Start timestamp in microseconds
            t_end_us: End timestamp in microseconds

        Returns:
            List of samples in time range (sorted by timestamp)
        """
        with self._lock:
            samples = [
                s for s in self._buffer
                if t_start_us <= s.timestamp_us <= t_end_us
            ]
        return sorted(samples, key=lambda s: s.timestamp_us)

    def get_latest(self, n: int = 1) -> list[IMUSample]:
        """Get most recent n samples.

        Args:
            n: Number of samples to retrieve

        Returns:
            List of most recent samples (oldest first)
        """
        with self._lock:
            if n >= len(self._buffer):
                return list(self._buffer)
            return list(self._buffer)[-n:]

    def get_since(self, timestamp_us: int) -> list[IMUSample]:
        """Get all samples since given timestamp.

        Args:
            timestamp_us: Start timestamp in microseconds

        Returns:
            List of samples after timestamp
        """
        with self._lock:
            samples = [s for s in self._buffer if s.timestamp_us > timestamp_us]
        return sorted(samples, key=lambda s: s.timestamp_us)

    def clear(self) -> None:
        """Clear all samples."""
        with self._lock:
            self._buffer.clear()

    @property
    def size(self) -> int:
        """Current number of samples in buffer."""
        with self._lock:
            return len(self._buffer)

    @property
    def total_samples(self) -> int:
        """Total samples received since creation."""
        return self._sample_count

    @property
    def latest_timestamp_us(self) -> Optional[int]:
        """Timestamp of most recent sample, or None if empty."""
        with self._lock:
            if self._buffer:
                return self._buffer[-1].timestamp_us
            return None

    @property
    def oldest_timestamp_us(self) -> Optional[int]:
        """Timestamp of oldest sample in buffer, or None if empty."""
        with self._lock:
            if self._buffer:
                return self._buffer[0].timestamp_us
            return None


class IMUHandler:
    """Process MAVLink IMU messages."""

    def __init__(self, config: IMUConfig):
        """Initialize IMU handler.

        Args:
            config: IMU configuration
        """
        self.config = config
        self._buffer = IMUBuffer(config.buffer_size)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._conn = None  # Set via set_connection
        self._callback: Optional[Callable[[IMUSample], None]] = None

        # Statistics
        self._last_sample_time = 0.0
        self._sample_rate = 0.0
        self._rate_window: deque[float] = deque(maxlen=100)

    def set_connection(self, conn) -> None:
        """Set MAVLink connection.

        Args:
            conn: pymavlink mavutil.mavlink_connection object
        """
        self._conn = conn

    def set_callback(self, callback: Callable[[IMUSample], None]) -> None:
        """Set callback for new IMU samples.

        Args:
            callback: Function called with each new IMUSample
        """
        self._callback = callback

    def start(self) -> bool:
        """Start background IMU reading thread.

        Returns:
            True if started successfully
        """
        if self._conn is None:
            logger.error("Cannot start IMU handler: no connection set")
            return False

        if self._running:
            logger.warning("IMU handler already running")
            return True

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("IMU handler started")
        return True

    def stop(self) -> None:
        """Stop IMU reading thread."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("IMU handler stopped")

    def _read_loop(self) -> None:
        """Background loop reading IMU messages."""
        while self._running:
            try:
                msg = self._conn.recv_match(
                    type=list(self.config.message_types),
                    blocking=True,
                    timeout=0.1,
                )
                if msg:
                    sample = self._parse_imu_message(msg)
                    if sample is not None:
                        self._buffer.add(sample)
                        self._update_rate()
                        if self._callback:
                            try:
                                self._callback(sample)
                            except Exception as e:
                                logger.error(f"IMU callback error: {e}")

            except Exception as e:
                if self._running:  # Only log if not shutting down
                    logger.error(f"Error reading IMU: {e}")
                time.sleep(0.01)

    def _parse_imu_message(self, msg) -> Optional[IMUSample]:
        """Convert MAVLink IMU message to IMUSample.

        Args:
            msg: MAVLink message (RAW_IMU, SCALED_IMU, or HIGHRES_IMU)

        Returns:
            IMUSample or None if parsing failed
        """
        try:
            msg_type = msg.get_type()

            if msg_type == "RAW_IMU":
                # RAW_IMU units:
                # xacc, yacc, zacc: mG (milli-g) -> convert to m/s^2
                # xgyro, ygyro, zgyro: mrad/s -> convert to rad/s
                accel = np.array(
                    [msg.xacc, msg.yacc, msg.zacc], dtype=np.float64
                ) * 0.001 * 9.81  # mG to m/s^2

                gyro = np.array(
                    [msg.xgyro, msg.ygyro, msg.zgyro], dtype=np.float64
                ) * 0.001  # mrad/s to rad/s

                return IMUSample(
                    timestamp_us=msg.time_usec,
                    accel=accel,
                    gyro=gyro,
                )

            elif msg_type == "SCALED_IMU":
                # SCALED_IMU units:
                # xacc, yacc, zacc: mg (milli-g) -> convert to m/s^2
                # xgyro, ygyro, zgyro: mrad/s -> convert to rad/s
                accel = np.array(
                    [msg.xacc, msg.yacc, msg.zacc], dtype=np.float64
                ) * 0.001 * 9.81  # mg to m/s^2

                gyro = np.array(
                    [msg.xgyro, msg.ygyro, msg.zgyro], dtype=np.float64
                ) * 0.001  # mrad/s to rad/s

                return IMUSample(
                    timestamp_us=msg.time_boot_ms * 1000,  # ms to us
                    accel=accel,
                    gyro=gyro,
                )

            elif msg_type == "HIGHRES_IMU":
                # HIGHRES_IMU units:
                # xacc, yacc, zacc: m/s^2 (already correct)
                # xgyro, ygyro, zgyro: rad/s (already correct)
                accel = np.array(
                    [msg.xacc, msg.yacc, msg.zacc], dtype=np.float64
                )

                gyro = np.array(
                    [msg.xgyro, msg.ygyro, msg.zgyro], dtype=np.float64
                )

                return IMUSample(
                    timestamp_us=msg.time_usec,
                    accel=accel,
                    gyro=gyro,
                )

            else:
                logger.warning(f"Unknown IMU message type: {msg_type}")
                return None

        except Exception as e:
            logger.error(f"Error parsing IMU message: {e}")
            return None

    def _update_rate(self) -> None:
        """Update sample rate estimate."""
        now = time.time()
        if self._last_sample_time > 0:
            dt = now - self._last_sample_time
            if dt > 0:
                self._rate_window.append(1.0 / dt)
                if len(self._rate_window) >= 10:
                    self._sample_rate = np.mean(self._rate_window)
        self._last_sample_time = now

    def add_sample(self, sample: IMUSample) -> None:
        """Manually add an IMU sample (for testing or external sources).

        Args:
            sample: IMU sample to add
        """
        self._buffer.add(sample)
        self._update_rate()
        if self._callback:
            self._callback(sample)

    def get_samples_between(
        self, t_start_us: int, t_end_us: int
    ) -> list[IMUSample]:
        """Get samples between timestamps.

        Args:
            t_start_us: Start timestamp in microseconds
            t_end_us: End timestamp in microseconds

        Returns:
            List of samples in range
        """
        return self._buffer.get_samples_between(t_start_us, t_end_us)

    def get_latest(self, n: int = 1) -> list[IMUSample]:
        """Get most recent samples.

        Args:
            n: Number of samples

        Returns:
            List of samples
        """
        return self._buffer.get_latest(n)

    def get_since(self, timestamp_us: int) -> list[IMUSample]:
        """Get samples since timestamp.

        Args:
            timestamp_us: Start timestamp

        Returns:
            List of samples after timestamp
        """
        return self._buffer.get_since(timestamp_us)

    def remove_bias(
        self, sample: IMUSample, accel_bias: np.ndarray, gyro_bias: np.ndarray
    ) -> IMUSample:
        """Remove bias from IMU sample.

        Args:
            sample: Original IMU sample
            accel_bias: Accelerometer bias to remove
            gyro_bias: Gyroscope bias to remove

        Returns:
            Bias-corrected IMU sample
        """
        return IMUSample(
            timestamp_us=sample.timestamp_us,
            accel=sample.accel - accel_bias,
            gyro=sample.gyro - gyro_bias,
        )

    @property
    def is_running(self) -> bool:
        """Whether handler is running."""
        return self._running

    @property
    def buffer_size(self) -> int:
        """Current buffer size."""
        return self._buffer.size

    @property
    def sample_rate(self) -> float:
        """Estimated sample rate in Hz."""
        return self._sample_rate

    @property
    def latest_timestamp_us(self) -> Optional[int]:
        """Timestamp of most recent sample."""
        return self._buffer.latest_timestamp_us


def integrate_imu(
    samples: list[IMUSample],
    initial_position: np.ndarray,
    initial_velocity: np.ndarray,
    initial_quaternion: np.ndarray,
    accel_bias: np.ndarray,
    gyro_bias: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate IMU samples to propagate state.

    Simple integration without EKF for dead reckoning fallback.

    Args:
        samples: List of IMU samples (sorted by time)
        initial_position: Starting position [x, y, z] in NED
        initial_velocity: Starting velocity [vx, vy, vz] in NED
        initial_quaternion: Starting orientation [w, x, y, z]
        accel_bias: Accelerometer bias to remove
        gyro_bias: Gyroscope bias to remove

    Returns:
        Tuple of (final_position, final_velocity, final_quaternion)
    """
    from .utils import (
        quaternion_from_rotation_vector,
        quaternion_multiply,
        rotation_matrix_from_quaternion,
    )

    position = initial_position.copy()
    velocity = initial_velocity.copy()
    quaternion = initial_quaternion.copy()

    gravity = np.array([0.0, 0.0, 9.81])  # NED frame, down is positive

    prev_timestamp_us = samples[0].timestamp_us if samples else 0

    for sample in samples:
        # Time delta
        dt = (sample.timestamp_us - prev_timestamp_us) / 1e6
        if dt <= 0 or dt > 0.1:  # Skip invalid or large gaps
            prev_timestamp_us = sample.timestamp_us
            continue

        # Remove bias
        accel = sample.accel - accel_bias
        gyro = sample.gyro - gyro_bias

        # Update orientation
        omega = gyro * dt
        dq = quaternion_from_rotation_vector(omega)
        quaternion = quaternion_multiply(quaternion, dq)
        quaternion = quaternion / np.linalg.norm(quaternion)

        # Rotate acceleration to NED frame
        R = rotation_matrix_from_quaternion(quaternion)
        accel_ned = R @ accel

        # Remove gravity
        accel_ned = accel_ned - gravity

        # Update velocity and position (simple Euler integration)
        velocity = velocity + accel_ned * dt
        position = position + velocity * dt + 0.5 * accel_ned * dt * dt

        prev_timestamp_us = sample.timestamp_us

    return position, velocity, quaternion
