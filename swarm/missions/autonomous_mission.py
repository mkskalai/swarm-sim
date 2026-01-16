"""Autonomous mission orchestrator for Phase 7.

Combines search patterns, pursuit behavior, GPS denial handling,
and semantic landmarks into cohesive autonomous missions.
"""

import time
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict, Set, Callable, Any

from ..coordination.search_patterns import (
    SearchPatternConfig,
    SearchPatternType,
    AdaptiveSearchController,
)
from ..coordination.pursuit_controller import (
    PursuitController,
    PursuitConfig,
    PursuitStrategy,
    PursuitTarget,
    SimulatedTarget,
)
from ..coordination.missions import PositionNED
from ..navigation.gps_jammer import GPSJammer, GPSJammerConfig, GPSDenialZone

logger = logging.getLogger(__name__)


class MissionPhase(Enum):
    """Phases of an autonomous mission."""
    IDLE = "idle"              # Not started
    LAUNCH = "launch"          # Taking off
    SEARCH = "search"          # Executing search pattern
    ACQUIRE = "acquire"        # Target acquired, transitioning to pursuit
    PURSUIT = "pursuit"        # Pursuing target
    LOITER = "loiter"         # Holding position
    RTL = "rtl"               # Return to launch
    EMERGENCY = "emergency"    # Emergency landing
    COMPLETE = "complete"      # Mission completed


@dataclass
class AutonomousMissionConfig:
    """Configuration for autonomous mission.

    Attributes:
        search_pattern: Search pattern configuration
        pursuit_config: Pursuit behavior configuration
        gps_jammer_config: GPS jamming configuration (optional)
        target_classes: Object classes to pursue when detected
        min_drones_for_pursuit: Minimum drones required to initiate pursuit
        search_timeout: Maximum search time before RTL (seconds)
        enable_semantic_landmarks: Use landmarks for VIO correction
        auto_transition_to_pursuit: Automatically pursue detected targets
        loiter_on_target_lost: Loiter at last known position vs search
        detection_confidence_threshold: Minimum confidence for pursuit
    """
    search_pattern: SearchPatternConfig
    pursuit_config: PursuitConfig = field(default_factory=PursuitConfig)
    gps_jammer_config: Optional[GPSJammerConfig] = None
    target_classes: List[str] = field(default_factory=lambda: ["person", "car"])
    min_drones_for_pursuit: int = 2
    search_timeout: float = 300.0
    enable_semantic_landmarks: bool = True
    auto_transition_to_pursuit: bool = True
    loiter_on_target_lost: bool = False
    detection_confidence_threshold: float = 0.6


@dataclass
class MissionState:
    """Current state of the mission.

    Attributes:
        phase: Current mission phase
        start_time: Mission start timestamp
        phase_start_time: Current phase start timestamp
        search_progress: Search completion percentage
        target_track_id: ID of target being pursued
        gps_denied_drones: Set of GPS-denied drone IDs
        detections_count: Total detections during mission
        waypoints_completed: Number of search waypoints completed
    """
    phase: MissionPhase = MissionPhase.IDLE
    start_time: float = 0.0
    phase_start_time: float = 0.0
    search_progress: float = 0.0
    target_track_id: Optional[int] = None
    gps_denied_drones: Set[int] = field(default_factory=set)
    detections_count: int = 0
    waypoints_completed: int = 0


class AutonomousMissionController:
    """Orchestrates autonomous swarm missions.

    Manages the mission state machine, coordinating between:
    - Search pattern execution
    - Target pursuit
    - GPS denial handling
    - Semantic landmark integration

    Example:
        config = AutonomousMissionConfig(
            search_pattern=SearchPatternConfig(
                pattern_type=SearchPatternType.EXPANDING_SQUARE,
                center=(50.0, 50.0),
                max_size=100.0,
            ),
            pursuit_config=PursuitConfig(
                strategy=PursuitStrategy.SURROUND,
            ),
        )

        controller = AutonomousMissionController(
            swarm_controller=swarm,
            config=config,
        )

        # Start mission
        controller.start_mission()

        # In main loop
        while not controller.is_complete:
            phase = controller.update(drone_positions, detections)
            targets = controller.get_current_targets()
            # Send targets to swarm controller
    """

    def __init__(
        self,
        swarm_controller=None,
        config: AutonomousMissionConfig = None,
        detection_tracker=None,
    ):
        """Initialize autonomous mission controller.

        Args:
            swarm_controller: SwarmController instance for drone control
            config: Mission configuration
            detection_tracker: Optional DetectionTracker for perception
        """
        self._swarm = swarm_controller
        self._config = config or AutonomousMissionConfig(
            search_pattern=SearchPatternConfig(
                pattern_type=SearchPatternType.EXPANDING_SQUARE,
                center=(0.0, 0.0),
                max_size=100.0,
            )
        )
        self._detection_tracker = detection_tracker

        # State
        self._state = MissionState()
        self._active_drones: Set[int] = set()

        # Sub-controllers
        self._search_controller: Optional[AdaptiveSearchController] = None
        self._pursuit_controller: Optional[PursuitController] = None
        self._gps_jammer: Optional[GPSJammer] = None

        # Target tracking
        self._current_target: Optional[PursuitTarget] = None
        self._simulated_target: Optional[SimulatedTarget] = None

        # Callbacks
        self._on_phase_change: List[Callable[[MissionPhase, MissionPhase], None]] = []
        self._on_target_detected: List[Callable[[PursuitTarget], None]] = []
        self._on_gps_denied: List[Callable[[int], None]] = []

        logger.info("Autonomous mission controller initialized")

    def start_mission(self, active_drones: Optional[List[int]] = None) -> bool:
        """Initialize and start the autonomous mission.

        Args:
            active_drones: List of active drone IDs (uses all if None)

        Returns:
            True if mission started successfully
        """
        if active_drones is None:
            # Default to drones 0-5 if no swarm controller
            active_drones = list(range(6))

        self._active_drones = set(active_drones)

        if len(self._active_drones) < 1:
            logger.error("No active drones for mission")
            return False

        # Initialize GPS jammer if configured
        if self._config.gps_jammer_config:
            self._gps_jammer = GPSJammer(self._config.gps_jammer_config)

        # Initialize search controller
        self._search_controller = AdaptiveSearchController(
            num_drones=len(self._active_drones),
            pattern_config=self._config.search_pattern,
        )
        self._search_controller.initialize_search(list(self._active_drones))

        # Initialize pursuit controller
        self._pursuit_controller = PursuitController(
            num_drones=len(self._active_drones),
            config=self._config.pursuit_config,
            detection_tracker=self._detection_tracker,
        )

        # Register callbacks
        self._search_controller.register_waypoint_callback(self._on_waypoint_reached)
        self._search_controller.register_completion_callback(self._on_search_complete)
        self._pursuit_controller.on_target_lost(self._on_pursuit_target_lost)

        # Start mission
        self._state.start_time = time.time()
        self._transition_to(MissionPhase.LAUNCH)

        logger.info(f"Mission started with {len(self._active_drones)} drones")
        return True

    def update(
        self,
        drone_positions: Dict[int, PositionNED],
        detections: Optional[List] = None,
        dt: float = 0.1,
    ) -> MissionPhase:
        """Main update loop - call at control rate.

        Args:
            drone_positions: Current positions of all drones
            detections: Optional list of detections from perception
            dt: Time delta since last update

        Returns:
            Current mission phase
        """
        current_time = time.time()

        # Update GPS jamming
        if self._gps_jammer:
            newly_jammed = self._gps_jammer.update(drone_positions, current_time)
            for drone_id in newly_jammed:
                self._state.gps_denied_drones.add(drone_id)
                self._handle_gps_denial(drone_id)

        # Update simulated target if present
        if self._simulated_target:
            self._simulated_target.update(dt)
            if self._pursuit_controller and self._state.phase == MissionPhase.PURSUIT:
                self._pursuit_controller.update_target_position(
                    self._simulated_target.position,
                    self._simulated_target.velocity,
                )

        # Phase-specific handling
        if self._state.phase == MissionPhase.LAUNCH:
            self._handle_launch_phase(drone_positions)

        elif self._state.phase == MissionPhase.SEARCH:
            self._handle_search_phase(drone_positions, detections)

        elif self._state.phase == MissionPhase.ACQUIRE:
            self._handle_acquire_phase(drone_positions)

        elif self._state.phase == MissionPhase.PURSUIT:
            self._handle_pursuit_phase(drone_positions, detections)

        elif self._state.phase == MissionPhase.LOITER:
            self._handle_loiter_phase(drone_positions)

        elif self._state.phase == MissionPhase.RTL:
            self._handle_rtl_phase(drone_positions)

        # Check mission timeout
        if self._state.phase == MissionPhase.SEARCH:
            elapsed = current_time - self._state.phase_start_time
            if elapsed > self._config.search_timeout:
                logger.warning("Search timeout reached, returning to launch")
                self._transition_to(MissionPhase.RTL)

        return self._state.phase

    def _handle_launch_phase(self, drone_positions: Dict[int, PositionNED]) -> None:
        """Handle launch phase - wait for takeoff completion."""
        # In simulation, assume drones are ready
        # In real deployment, would check altitudes
        all_airborne = all(
            pos[2] < -3.0  # At least 3m altitude (down is negative)
            for pos in drone_positions.values()
        )

        if all_airborne or time.time() - self._state.phase_start_time > 30.0:
            self._transition_to(MissionPhase.SEARCH)

    def _handle_search_phase(
        self,
        drone_positions: Dict[int, PositionNED],
        detections: Optional[List],
    ) -> None:
        """Handle search phase - execute search pattern."""
        if self._search_controller is None:
            return

        # Update search progress
        for drone_id, position in drone_positions.items():
            self._search_controller.update_progress(drone_id, position)

        # Check for target detections
        if detections and self._config.auto_transition_to_pursuit:
            target = self._check_for_pursuit_target(detections)
            if target:
                self._current_target = target
                self._state.target_track_id = target.target_id
                self._transition_to(MissionPhase.ACQUIRE)

        # Update stats
        stats = self._search_controller.coverage_stats
        self._state.search_progress = stats["completion_percent"]
        self._state.waypoints_completed = int(stats["completed_waypoints"])

        # Check if search complete
        if self._search_controller.is_search_complete:
            logger.info("Search pattern complete, transitioning to RTL")
            self._transition_to(MissionPhase.RTL)

    def _handle_acquire_phase(self, drone_positions: Dict[int, PositionNED]) -> None:
        """Handle acquisition phase - assign pursuers."""
        if self._pursuit_controller is None or self._current_target is None:
            self._transition_to(MissionPhase.SEARCH)
            return

        # Set target in pursuit controller
        self._pursuit_controller.set_target(self._current_target)

        # Assign pursuers
        available = [d for d in self._active_drones if d not in self._state.gps_denied_drones]

        if len(available) >= self._config.min_drones_for_pursuit:
            self._pursuit_controller.assign_pursuers(available)
            self._transition_to(MissionPhase.PURSUIT)

            for callback in self._on_target_detected:
                callback(self._current_target)
        else:
            logger.warning(f"Not enough drones for pursuit ({len(available)} < {self._config.min_drones_for_pursuit})")
            self._transition_to(MissionPhase.SEARCH)

    def _handle_pursuit_phase(
        self,
        drone_positions: Dict[int, PositionNED],
        detections: Optional[List],
    ) -> None:
        """Handle pursuit phase - track target."""
        if self._pursuit_controller is None:
            return

        # Update target from detections if available
        if detections and self._state.target_track_id is not None:
            for det in detections:
                if hasattr(det, 'track_id') and det.track_id == self._state.target_track_id:
                    self._pursuit_controller.update_target_position(
                        det.estimated_position_ned if hasattr(det, 'estimated_position_ned') else det.position,
                    )
                    break

        # Check if target still visible
        if not self._pursuit_controller.check_target_visibility(drone_positions):
            self._on_pursuit_target_lost(self._current_target)

    def _handle_loiter_phase(self, drone_positions: Dict[int, PositionNED]) -> None:
        """Handle loiter phase - hold position."""
        # Loiter for a fixed time then resume search or RTL
        elapsed = time.time() - self._state.phase_start_time

        if elapsed > 30.0:  # 30 second loiter
            if self._search_controller and not self._search_controller.is_search_complete:
                self._transition_to(MissionPhase.SEARCH)
            else:
                self._transition_to(MissionPhase.RTL)

    def _handle_rtl_phase(self, drone_positions: Dict[int, PositionNED]) -> None:
        """Handle return to launch phase."""
        # Check if all drones near origin
        all_home = all(
            abs(pos[0]) < 5.0 and abs(pos[1]) < 5.0
            for pos in drone_positions.values()
        )

        if all_home:
            self._transition_to(MissionPhase.COMPLETE)

    def _check_for_pursuit_target(self, detections: List) -> Optional[PursuitTarget]:
        """Check detections for valid pursuit target.

        Args:
            detections: List of detections

        Returns:
            PursuitTarget if valid target found
        """
        for det in detections:
            class_name = det.class_name if hasattr(det, 'class_name') else str(det.class_id)
            confidence = det.confidence if hasattr(det, 'confidence') else 0.0

            if class_name in self._config.target_classes:
                if confidence >= self._config.detection_confidence_threshold:
                    position = (
                        det.estimated_position_ned
                        if hasattr(det, 'estimated_position_ned')
                        else (0.0, 0.0, 0.0)
                    )

                    return PursuitTarget(
                        target_id=det.track_id if hasattr(det, 'track_id') else 1,
                        position=position,
                        class_name=class_name,
                        confidence=confidence,
                        last_seen_time=time.time(),
                        seen_by_drones={det.drone_id} if hasattr(det, 'drone_id') else set(),
                    )

        return None

    def _on_waypoint_reached(self, drone_id: int, waypoint) -> None:
        """Callback when drone reaches search waypoint."""
        logger.debug(f"Drone {drone_id} reached waypoint")
        self._state.detections_count += 1

    def _on_search_complete(self) -> None:
        """Callback when search pattern complete."""
        logger.info("Search pattern completed")

    def _on_pursuit_target_lost(self, target: Optional[PursuitTarget]) -> None:
        """Callback when pursuit target is lost."""
        logger.info("Pursuit target lost")

        if self._config.loiter_on_target_lost:
            self._transition_to(MissionPhase.LOITER)
        else:
            # Get search pattern from pursuit controller
            if self._pursuit_controller:
                search_config = self._pursuit_controller.handle_target_lost()
                if search_config and self._search_controller:
                    # Reinitialize search with new pattern
                    self._search_controller = AdaptiveSearchController(
                        num_drones=len(self._active_drones),
                        pattern_config=search_config,
                    )
                    self._search_controller.initialize_search(list(self._active_drones))

            self._transition_to(MissionPhase.SEARCH)

        self._current_target = None
        self._state.target_track_id = None

    def _handle_gps_denial(self, drone_id: int) -> None:
        """Handle GPS denial for a drone."""
        logger.warning(f"Drone {drone_id} GPS denied, switching to VIO")

        for callback in self._on_gps_denied:
            callback(drone_id)

        # If searching, reallocate search area
        if self._state.phase == MissionPhase.SEARCH and self._search_controller:
            # GPS-denied drones can still search but with reduced reliability
            pass

    def _transition_to(self, new_phase: MissionPhase) -> None:
        """Transition to new mission phase."""
        old_phase = self._state.phase
        self._state.phase = new_phase
        self._state.phase_start_time = time.time()

        logger.info(f"Mission phase: {old_phase.value} -> {new_phase.value}")

        for callback in self._on_phase_change:
            callback(old_phase, new_phase)

    def get_current_targets(self) -> Dict[int, PositionNED]:
        """Get current target positions for all drones.

        Returns:
            Dictionary mapping drone_id to target position
        """
        if self._state.phase == MissionPhase.SEARCH:
            if self._search_controller:
                return self._search_controller.get_current_targets()

        elif self._state.phase in (MissionPhase.ACQUIRE, MissionPhase.PURSUIT):
            if self._pursuit_controller:
                return self._pursuit_controller.get_pursuit_targets({})

        elif self._state.phase == MissionPhase.RTL:
            # Return to origin
            return {d: (0.0, 0.0, -10.0) for d in self._active_drones}

        return {}

    def set_simulated_target(self, target: SimulatedTarget) -> None:
        """Set simulated target for testing.

        Args:
            target: SimulatedTarget instance
        """
        self._simulated_target = target

    def abort_mission(self) -> None:
        """Abort mission and return to launch."""
        logger.warning("Mission aborted")
        self._transition_to(MissionPhase.RTL)

    def emergency_land(self) -> None:
        """Trigger emergency landing."""
        logger.error("Emergency landing initiated")
        self._transition_to(MissionPhase.EMERGENCY)

    @property
    def phase(self) -> MissionPhase:
        """Current mission phase."""
        return self._state.phase

    @property
    def is_complete(self) -> bool:
        """Whether mission is complete."""
        return self._state.phase == MissionPhase.COMPLETE

    @property
    def is_active(self) -> bool:
        """Whether mission is actively running."""
        return self._state.phase not in (
            MissionPhase.IDLE,
            MissionPhase.COMPLETE,
            MissionPhase.EMERGENCY,
        )

    @property
    def state(self) -> MissionState:
        """Current mission state."""
        return self._state

    @property
    def stats(self) -> Dict[str, Any]:
        """Get mission statistics."""
        elapsed = time.time() - self._state.start_time if self._state.start_time else 0

        return {
            "phase": self._state.phase.value,
            "elapsed_time": elapsed,
            "search_progress": self._state.search_progress,
            "waypoints_completed": self._state.waypoints_completed,
            "detections": self._state.detections_count,
            "gps_denied_drones": len(self._state.gps_denied_drones),
            "active_drones": len(self._active_drones),
            "tracking_target": self._state.target_track_id is not None,
        }

    def on_phase_change(
        self,
        callback: Callable[[MissionPhase, MissionPhase], None],
    ) -> None:
        """Register callback for phase changes."""
        self._on_phase_change.append(callback)

    def on_target_detected(self, callback: Callable[[PursuitTarget], None]) -> None:
        """Register callback for target detection."""
        self._on_target_detected.append(callback)

    def on_gps_denied(self, callback: Callable[[int], None]) -> None:
        """Register callback for GPS denial events."""
        self._on_gps_denied.append(callback)
