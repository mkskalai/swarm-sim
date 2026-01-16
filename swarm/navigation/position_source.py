"""Position source for hybrid GPS/VIO navigation."""

import logging
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .config import NavigationConfig, NavigationMode
from .vio_estimator import VIOEstimator, VIOState

logger = logging.getLogger(__name__)


@dataclass
class GPSData:
    """GPS measurement data."""

    position: np.ndarray  # [north, east, down] in meters
    velocity: Optional[np.ndarray]  # [vn, ve, vd] in m/s
    timestamp: float  # Unix timestamp
    fix_type: int  # GPS fix type (0=no fix, 3=3D fix)
    hdop: float  # Horizontal dilution of precision

    @property
    def is_valid(self) -> bool:
        """Whether GPS data is valid for navigation."""
        return self.fix_type >= 3 and self.hdop < 5.0


class PositionSource:
    """Hybrid GPS/VIO position provider.

    Automatically switches between GPS and VIO based on availability
    and quality of each source.
    """

    def __init__(self, config: NavigationConfig, drone_id: int = 0):
        """Initialize position source.

        Args:
            config: Navigation configuration
            drone_id: Drone identifier
        """
        self.config = config
        self.drone_id = drone_id

        # VIO estimator
        self._vio = VIOEstimator(config, drone_id)

        # GPS state
        self._last_gps: Optional[GPSData] = None
        self._gps_available = False
        self._gps_denied_start: Optional[float] = None

        # Mode tracking
        self._current_mode = NavigationMode.GPS
        self._mode_switch_time: float = 0.0

        # Statistics
        self._gps_denied_duration: float = 0.0
        self._total_vio_distance: float = 0.0

    def initialize(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        attitude: np.ndarray,
        altitude: float,
        timestamp_us: int,
    ) -> bool:
        """Initialize position source.

        Args:
            position: Initial position [north, east, down]
            velocity: Initial velocity [vn, ve, vd]
            attitude: Initial attitude [roll, pitch, yaw] or quaternion
            altitude: Initial altitude for scale
            timestamp_us: Initial timestamp

        Returns:
            True if initialized successfully
        """
        if not self._vio.initialize(
            position, velocity, attitude, altitude, timestamp_us
        ):
            return False

        self._gps_available = True
        self._current_mode = NavigationMode.GPS
        logger.info(f"Position source initialized for drone {self.drone_id}")
        return True

    def set_mavlink_connection(self, conn) -> None:
        """Set MAVLink connection for IMU reading."""
        self._vio.set_mavlink_connection(conn)

    def start(self) -> bool:
        """Start VIO processing (IMU handler)."""
        return self._vio.start_imu_handler()

    def stop(self) -> None:
        """Stop VIO processing."""
        self._vio.stop_imu_handler()

    def update_gps(
        self,
        position: np.ndarray,
        velocity: Optional[np.ndarray] = None,
        fix_type: int = 3,
        hdop: float = 1.0,
        timestamp: Optional[float] = None,
    ) -> None:
        """Update with GPS measurement.

        Args:
            position: GPS position [north, east, down]
            velocity: GPS velocity [vn, ve, vd] (optional)
            fix_type: GPS fix type
            hdop: Horizontal dilution of precision
            timestamp: GPS timestamp (uses current time if None)
        """
        if timestamp is None:
            timestamp = time.time()

        self._last_gps = GPSData(
            position=np.asarray(position),
            velocity=np.asarray(velocity) if velocity is not None else None,
            timestamp=timestamp,
            fix_type=fix_type,
            hdop=hdop,
        )

        if self._last_gps.is_valid:
            # GPS available
            if not self._gps_available:
                # GPS just recovered
                if self._gps_denied_start is not None:
                    denied_duration = timestamp - self._gps_denied_start
                    self._gps_denied_duration += denied_duration
                    logger.info(
                        f"GPS recovered after {denied_duration:.1f}s denial "
                        f"(total denied: {self._gps_denied_duration:.1f}s)"
                    )
                self._gps_denied_start = None

            self._gps_available = True

            # Update VIO with GPS for fusion/correction
            if self.config.mode == NavigationMode.HYBRID:
                self._vio.update_with_gps(position, velocity, timestamp)

        else:
            self._gps_available = False

    def process_image(
        self, frame: np.ndarray, timestamp_us: int
    ) -> Optional[VIOState]:
        """Process camera image for VIO.

        Args:
            frame: Camera image
            timestamp_us: Image timestamp

        Returns:
            VIO state estimate
        """
        return self._vio.process_image(frame, timestamp_us)

    def get_position(self) -> Tuple[Optional[np.ndarray], NavigationMode]:
        """Get best available position estimate.

        Returns:
            Tuple of (position_ned, current_mode)
        """
        mode = self._determine_mode()

        if mode == NavigationMode.GPS:
            if self._last_gps is not None:
                return self._last_gps.position.copy(), NavigationMode.GPS
            else:
                # No GPS yet, try VIO
                return self._get_vio_position()

        elif mode == NavigationMode.VIO or mode == NavigationMode.HYBRID:
            return self._get_vio_position()

        else:
            # Dead reckoning
            return self._get_vio_position()

    def get_state(self) -> Optional[VIOState]:
        """Get full VIO state estimate.

        Returns:
            VIO state or None
        """
        return self._vio.get_state()

    def _get_vio_position(self) -> Tuple[Optional[np.ndarray], NavigationMode]:
        """Get position from VIO."""
        pos, mode = self._vio.get_position()
        return pos, mode

    def _determine_mode(self) -> NavigationMode:
        """Determine current navigation mode."""
        now = time.time()

        # Check configured mode
        if self.config.mode == NavigationMode.GPS:
            if self._check_gps_available():
                return NavigationMode.GPS
            else:
                logger.warning("GPS mode configured but GPS unavailable")
                return NavigationMode.DEAD_RECKONING

        elif self.config.mode == NavigationMode.VIO:
            return NavigationMode.VIO

        # Hybrid mode logic
        gps_ok = self._check_gps_available()
        vio_ok = self._vio.is_healthy

        if gps_ok and vio_ok:
            return NavigationMode.HYBRID
        elif gps_ok:
            return NavigationMode.GPS
        elif vio_ok:
            # GPS denied, start tracking
            if self._gps_denied_start is None:
                self._gps_denied_start = now
                logger.warning(f"GPS denied for drone {self.drone_id}, switching to VIO")
            return NavigationMode.VIO
        else:
            return NavigationMode.DEAD_RECKONING

    def _check_gps_available(self) -> bool:
        """Check if GPS is available and recent."""
        if self._last_gps is None:
            return False

        if not self._last_gps.is_valid:
            return False

        time_since_gps = time.time() - self._last_gps.timestamp
        return time_since_gps < self.config.gps_timeout

    def deny_gps(self) -> None:
        """Simulate GPS denial (for testing)."""
        self._gps_available = False
        self._gps_denied_start = time.time()
        logger.info(f"GPS denied for drone {self.drone_id}")

    def restore_gps(self) -> None:
        """Restore GPS (for testing)."""
        if self._last_gps is not None:
            self._gps_available = True
            if self._gps_denied_start is not None:
                denied_duration = time.time() - self._gps_denied_start
                self._gps_denied_duration += denied_duration
                self._gps_denied_start = None
            logger.info(f"GPS restored for drone {self.drone_id}")

    def reset(self) -> None:
        """Reset position source."""
        self._vio.reset()
        self._last_gps = None
        self._gps_available = False
        self._gps_denied_start = None
        self._current_mode = NavigationMode.GPS
        logger.info(f"Position source reset for drone {self.drone_id}")

    @property
    def current_mode(self) -> NavigationMode:
        """Current navigation mode."""
        return self._determine_mode()

    @property
    def is_gps_available(self) -> bool:
        """Whether GPS is currently available."""
        return self._check_gps_available()

    @property
    def is_vio_healthy(self) -> bool:
        """Whether VIO is healthy."""
        return self._vio.is_healthy

    @property
    def gps_denied_duration(self) -> float:
        """Total time spent in GPS-denied mode (seconds)."""
        if self._gps_denied_start is not None:
            return self._gps_denied_duration + (time.time() - self._gps_denied_start)
        return self._gps_denied_duration

    @property
    def vio_quality(self) -> float:
        """VIO quality metric (0-1)."""
        return self._vio.vo_quality

    @property
    def scale_confidence(self) -> float:
        """Scale estimation confidence (0-1)."""
        return self._vio.scale_confidence


class GPSDenialSimulator:
    """Simulate GPS denial for testing.

    Wraps a PositionSource and allows controlled GPS denial scenarios.
    """

    def __init__(self):
        """Initialize simulator."""
        self._sources: dict[int, PositionSource] = {}
        self._denied_drones: set[int] = set()

    def register_source(self, drone_id: int, source: PositionSource) -> None:
        """Register a position source.

        Args:
            drone_id: Drone identifier
            source: Position source to register
        """
        self._sources[drone_id] = source

    def deny_gps(self, drone_id: int) -> None:
        """Deny GPS for specified drone.

        Args:
            drone_id: Drone to deny GPS
        """
        if drone_id in self._sources:
            self._sources[drone_id].deny_gps()
            self._denied_drones.add(drone_id)

    def deny_gps_all(self) -> None:
        """Deny GPS for all registered drones."""
        for drone_id in self._sources:
            self.deny_gps(drone_id)

    def restore_gps(self, drone_id: int) -> None:
        """Restore GPS for specified drone.

        Args:
            drone_id: Drone to restore GPS
        """
        if drone_id in self._sources:
            self._sources[drone_id].restore_gps()
            self._denied_drones.discard(drone_id)

    def restore_gps_all(self) -> None:
        """Restore GPS for all drones."""
        for drone_id in list(self._denied_drones):
            self.restore_gps(drone_id)

    def is_gps_denied(self, drone_id: int) -> bool:
        """Check if GPS is denied for drone.

        Args:
            drone_id: Drone to check

        Returns:
            True if GPS is denied
        """
        return drone_id in self._denied_drones

    @property
    def denied_drones(self) -> set[int]:
        """Set of drones with denied GPS."""
        return self._denied_drones.copy()
