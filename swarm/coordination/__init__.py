"""Coordination modules for swarm behaviors.

This package provides:
- Formation flying (line, grid, star)
- Leader-follower mode with automatic leader promotion
- Waypoint mission planning and execution
- Failure detection and graceful degradation
- Main SwarmController orchestrator using pymavlink
"""

from .formations import (
    FormationType,
    FormationConfig,
    FormationCalculator,
    FormationTransition,
    get_formation_positions,
    PositionNED,
)

from .leader_follower import (
    LeaderFollowerController,
    LeaderState,
    FollowerOffset,
    DroneStatus,
)

from .missions import (
    Waypoint,
    WaypointAction,
    DroneMission,
    SwarmMission,
    MissionExecutionMode,
    MissionPlanner,
    MissionExecutor,
)

from .failure_handler import (
    FailureHandler,
    FailureType,
    FailureEvent,
    DroneHealth,
)

from .swarm_controller import (
    SwarmController,
    SwarmConfig,
)

from .search_patterns import (
    SearchPatternGenerator,
    SearchPatternType,
    SearchPatternConfig,
    SearchProgress,
    AdaptiveSearchController,
)

from .pursuit_controller import (
    PursuitController,
    PursuitStrategy,
    PursuitConfig,
    SimulatedTarget,
)

__all__ = [
    # Formations
    "FormationType",
    "FormationConfig",
    "FormationCalculator",
    "FormationTransition",
    "get_formation_positions",
    "PositionNED",
    # Leader-Follower
    "LeaderFollowerController",
    "LeaderState",
    "FollowerOffset",
    "DroneStatus",
    # Missions
    "Waypoint",
    "WaypointAction",
    "DroneMission",
    "SwarmMission",
    "MissionExecutionMode",
    "MissionPlanner",
    "MissionExecutor",
    # Failure Handling
    "FailureHandler",
    "FailureType",
    "FailureEvent",
    "DroneHealth",
    # Controller
    "SwarmController",
    "SwarmConfig",
    # Search Patterns
    "SearchPatternGenerator",
    "SearchPatternType",
    "SearchPatternConfig",
    "SearchProgress",
    "AdaptiveSearchController",
    # Pursuit
    "PursuitController",
    "PursuitStrategy",
    "PursuitConfig",
    "SimulatedTarget",
]
