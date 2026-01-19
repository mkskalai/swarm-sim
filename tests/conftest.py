"""Shared pytest configuration and fixtures for swarm tests.

This module provides:
- Custom markers for test categorization
- Shared fixtures for simulation tests
- Environment detection utilities
"""

import logging
import os
import pytest
from pathlib import Path

logger = logging.getLogger(__name__)

# Project root for imports
PROJECT_ROOT = Path(__file__).parent.parent


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "unit: Pure Python tests, no simulation required")
    config.addinivalue_line("markers", "sim: Full stack simulation tests (ROS2 + Gazebo + SITL)")
    config.addinivalue_line("markers", "gpu: Tests requiring GPU (YOLO detection)")
    config.addinivalue_line("markers", "slow: Tests that take a long time")


# Environment detection
def is_sim_running() -> bool:
    """Check if simulation is running."""
    return os.environ.get("SWARM_SIM_RUNNING", "").lower() in ("1", "true", "yes")


SIM_RUNNING = is_sim_running()
SKIP_SIM_REASON = "Requires running simulation. Use: python scripts/run_tests.py --sim"


@pytest.fixture
def num_drones() -> int:
    """Get drone count from environment."""
    return int(os.environ.get("SWARM_NUM_DRONES", "3"))


@pytest.fixture
def base_port() -> int:
    """Get base UDP port from environment."""
    return int(os.environ.get("SWARM_BASE_PORT", "14540"))


@pytest.fixture
def timeout_multiplier() -> float:
    """Get timeout multiplier from environment.

    Use --timeout-multiplier=N with run_tests.py to increase all timeouts.
    Useful for CPU-only systems or slow hardware (use 2.0-3.0x).
    """
    return float(os.environ.get("SWARM_TIMEOUT_MULTIPLIER", "1.0"))


@pytest.fixture
def swarm_config(num_drones):
    """Create SwarmConfig for tests."""
    from swarm.coordination import SwarmConfig
    return SwarmConfig(num_drones=num_drones)


@pytest.fixture
def swarm_controller(num_drones, base_port, timeout_multiplier):
    """Create connected SwarmController for simulation tests.

    This fixture connects to running SITL instances and yields
    a ready-to-use controller. Automatically disconnects on teardown.

    Timeouts are multiplied by timeout_multiplier (from --timeout-multiplier).
    """
    if not SIM_RUNNING:
        pytest.skip(SKIP_SIM_REASON)

    from swarm.coordination import SwarmController, SwarmConfig

    # Apply timeout multiplier to base timeouts
    connection_timeout = 30.0 * timeout_multiplier
    ekf_timeout = 60.0 * timeout_multiplier

    config = SwarmConfig(
        num_drones=num_drones,
        base_port=base_port,
        connection_timeout=connection_timeout,
        ekf_timeout=ekf_timeout,
    )
    controller = SwarmController(config)

    try:
        controller.connect_all(timeout=connection_timeout)
        controller.wait_for_ekf_all(timeout=ekf_timeout)
        yield controller
    finally:
        controller.disconnect_all()


@pytest.fixture
def armed_swarm(swarm_controller, timeout_multiplier):
    """Create armed SwarmController ready for flight.

    This fixture builds on swarm_controller and arms all drones.
    Automatically lands and disarms on teardown with proper verification.
    """
    swarm_controller.set_mode_all("GUIDED")
    swarm_controller.arm_all()

    yield swarm_controller

    # Teardown: land with verification and proper error handling
    landing_timeout = 60.0 * timeout_multiplier
    try:
        # Use spread_and_land to avoid collision during descent
        landed = swarm_controller.spread_and_land(
            spacing=8.0,
            spread_timeout=30.0 * timeout_multiplier,
            landing_timeout=landing_timeout,
            altitude_threshold=1.0,
        )
        if not landed:
            logger.warning("Not all drones verified landed during teardown")
    except Exception as e:
        logger.error(f"Landing failed during teardown: {e}")
        # Force disarm as last resort
        try:
            swarm_controller.disarm_all()
        except Exception:
            pass


@pytest.fixture
def flying_swarm(armed_swarm):
    """Create SwarmController with drones in flight.

    This fixture builds on armed_swarm and takes off all drones.
    Automatically lands on teardown.
    """
    armed_swarm.takeoff_all(altitude=5.0)

    yield armed_swarm

    # Teardown handled by armed_swarm fixture


# Skip decorators for convenience
skip_without_sim = pytest.mark.skipif(not SIM_RUNNING, reason=SKIP_SIM_REASON)


def pytest_collection_modifyitems(config, items):
    """Auto-add markers based on test location."""
    for item in items:
        # Auto-mark tests in tests/unit/ with @pytest.mark.unit
        if "tests/unit" in str(item.fspath):
            item.add_marker(pytest.mark.unit)

        # Auto-mark tests in tests/simulation/ with @pytest.mark.sim
        if "tests/simulation" in str(item.fspath):
            item.add_marker(pytest.mark.sim)
