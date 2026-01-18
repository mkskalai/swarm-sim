# Test Infrastructure Refactoring Plan

## Executive Summary

The current test infrastructure has grown organically across "phases" with significant fragmentation, duplication, and architectural drift. This plan consolidates everything into a clean, maintainable structure.

**Key Problems:**
1. "Phase" naming is meaningless to external users
2. MAVSDK-based classes (Fleet, Drone) are deprecated but still referenced
3. ~600 lines of process management code duplicated across test files
4. Tests mixed with infrastructure (SITL spawning in test scripts)
5. 6+ world files when we need 2 (basic + complex)
6. No clear developer documentation

**Final Decisions:**
1. GPS denial zones: **Runtime config** (Python GPSJammer class), not in world files
2. Drone count: **No cap**, auto-generate any number, **grid spawn by default** (sqrt x sqrt)
3. ROS2: **Default for simulation tests**. `--no-ros` flag for edge cases only
4. Deprecation: **None** - just delete Fleet/Drone immediately (no external users)
5. CI: **Unit tests only in GitHub Actions**, simulation tests local/Docker only
6. Integration = **Full stack**: ROS2 + Gazebo + SITL + formations + GPS-denied + YOLO together

---

## Current State Analysis

### Files to DELETE (deprecated/redundant)

| File | Reason |
|------|--------|
| `swarm/core/fleet.py` | Uses MAVSDK, replaced by SwarmController |
| `swarm/core/drone.py` | Uses MAVSDK, replaced by SwarmController |
| `scripts/test_connection.py` | Uses MAVSDK Drone class |
| `scripts/test_offboard.py` | Uses MAVSDK Drone class |
| `scripts/test_fleet.py` | Uses MAVSDK Fleet class |
| `tests/test_phase1.py` | Tests deprecated Drone/DroneConfig |
| `worlds/single_drone.sdf` | Not needed, use basic ardupilot world |
| `worlds/multi_drone_3.sdf` | Generate dynamically |
| `worlds/multi_drone_4.sdf` | Generate dynamically |
| `worlds/multi_drone_6.sdf` | Generate dynamically |
| `worlds/terrain_urban_4.sdf` | Consolidate into one complex world |

### Files to RENAME/REFACTOR

| Current | New | Notes |
|---------|-----|-------|
| `tests/test_phase2.py` | `tests/unit/test_config.py` | Keep only config/unit tests |
| `tests/test_phase3.py` | `tests/unit/test_formations.py` | Already descriptive |
| `scripts/test_phase3.py` | DELETE | Merge into simulation framework |
| `scripts/test_phase5.py` | DELETE | Merge into simulation framework |
| `scripts/test_phase6.py` | DELETE | Merge into simulation framework |
| `scripts/test_phase7.py` | DELETE | Merge into simulation framework |
| `scripts/run_phase3_test.py` | `swarm/simulation/sim_manager.py` | Extract infrastructure |
| `scripts/run_integration_tests.py` | `scripts/run_tests.py` | Single entry point |

---

## Proposed Architecture

### Directory Structure

```
swarm/
├── coordination/           # Keep: SwarmController, formations, etc.
├── navigation/             # Keep: VIO, EKF, GPS jammer
├── perception/             # Keep: YOLO detector, tracker
├── missions/               # Keep: autonomous missions
├── simulation/             # NEW: Infrastructure extracted from tests
│   ├── __init__.py
│   ├── sim_manager.py      # Gazebo + SITL + ROS2 orchestration
│   ├── world_generator.py  # Dynamic world/model generation
│   └── sitl_launcher.py    # SITL process management
└── core/
    ├── __init__.py
    └── config.py           # Keep: SwarmConfig (remove FleetConfig, DroneConfig)

scripts/
├── run_tests.py            # Single entry point for all tests
├── generate_world.py       # Keep: world generation CLI
└── (other utilities)

tests/
├── conftest.py             # Shared fixtures, markers
├── unit/                   # Fast tests, no simulation (runs in GHA)
│   ├── test_config.py
│   ├── test_formations.py
│   ├── test_navigation.py
│   ├── test_search_patterns.py
│   ├── test_pursuit.py
│   ├── test_gps_jammer.py
│   └── test_perception.py  # YOLO/tracker unit tests (CPU mode)
└── simulation/             # Full stack: ROS2 + Gazebo + SITL (local/Docker only)
    └── test_swarm.py       # All scenarios in one file

worlds/
├── basic.sdf               # Minimal world (ground plane, sun)
└── complex.sdf             # People, trucks, buildings for perception

models/
├── DroneTemplate/          # Keep: template for generation
└── (generated models)      # Drone1, Drone2, etc. auto-generated on demand
```

### Core Classes (After Refactor)

| Class | Location | Purpose |
|-------|----------|---------|
| `SwarmController` | `swarm/coordination/swarm_controller.py` | Main orchestrator (pymavlink) - UNCHANGED |
| `SwarmConfig` | `swarm/core/config.py` | Port calculations, spawn positions |
| `SimManager` | `swarm/simulation/sim_manager.py` | NEW: Gazebo + SITL + ROS2 orchestration |
| `WorldGenerator` | `swarm/simulation/world_generator.py` | NEW: Dynamic world/model generation |
| `SITLLauncher` | `swarm/simulation/sitl_launcher.py` | NEW: Extracted from launch_sitl.py |

### Deleted Classes

| Class | Reason |
|-------|--------|
| `Fleet` | MAVSDK-based, doesn't work for multi-drone |
| `Drone` | MAVSDK-based, replaced by SwarmController |
| `DroneConfig` | Only used by deleted Drone class |
| `FleetConfig` | Functionality merged into SwarmConfig |

---

## New Test Organization

### Test Categories

| Category | Description | Where it runs | Markers |
|----------|-------------|---------------|---------|
| **Unit** | Pure Python logic, no external deps | GitHub Actions + local | `@pytest.mark.unit` |
| **Simulation** | Full stack: ROS2 + Gazebo + SITL | Local / Docker only | `@pytest.mark.sim` |

```python
# tests/conftest.py

import pytest
import os

# Markers
def pytest_configure(config):
    config.addinivalue_line("markers", "unit: Pure Python, no simulation")
    config.addinivalue_line("markers", "sim: Full stack simulation (ROS2 + Gazebo + SITL)")

# Environment detection
SIM_RUNNING = os.environ.get("SWARM_SIM_RUNNING") == "1"

@pytest.fixture
def num_drones():
    """Get drone count from environment."""
    return int(os.environ.get("SWARM_NUM_DRONES", "3"))

@pytest.fixture
def swarm_controller(num_drones):
    """Create connected SwarmController for simulation tests."""
    from swarm.coordination import SwarmController, SwarmConfig

    config = SwarmConfig(num_drones=num_drones)
    controller = SwarmController(config)
    controller.connect_all()
    controller.wait_for_ekf_all()
    yield controller
    controller.disconnect_all()
```

### Descriptive Test Names

**Instead of:** `test_phase2.py::TestFleetIntegration::test_fleet_connection`

**Use:** `test_swarm.py::test_formation_flight_with_gps_denial`

### What Simulation Tests Cover (All Together)

A single simulation test run exercises the **full stack**:

1. Spawn N drones (grid layout) via ROS2 launch
2. Connect via SwarmBridge → pymavlink → SITL
3. Takeoff and fly formation (LINE, V, GRID, etc.)
4. Enable GPS denial on subset mid-flight (GPSJammer)
5. Affected drones switch to VIO (Phase 6 navigation)
6. YOLO detection running on complex world (people, trucks)
7. Execute search pattern with detection callbacks
8. Land all drones

This is ONE test scenario, not split by "phase" or feature.

---

## Single Test Entry Point

### Usage

```bash
# Run all unit tests (fast, no simulation) - also runs in GitHub Actions
python scripts/run_tests.py --unit

# Run simulation tests (default: ROS2 + Gazebo + SITL)
python scripts/run_tests.py --sim -n 3

# Run simulation with 6 drones, complex world
python scripts/run_tests.py --sim -n 6 --world complex

# Run simulation WITHOUT ROS2 (edge case for algorithm testing)
python scripts/run_tests.py --sim -n 3 --no-ros

# Run in Docker
./docker/scripts/run.sh test --sim -n 6

# Skip simulation startup (assume already running)
python scripts/run_tests.py --sim -n 3 --skip-sim

# Run everything (unit + simulation)
python scripts/run_tests.py --all -n 6
```

### Implementation

```python
# scripts/run_tests.py

import argparse
import pytest
import sys
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from swarm.simulation import SimManager

def main():
    parser = argparse.ArgumentParser(description="Run swarm tests")

    # Test selection
    parser.add_argument("--unit", action="store_true", help="Run unit tests only (fast, no deps)")
    parser.add_argument("--sim", action="store_true", help="Run simulation tests (ROS2 + Gazebo + SITL)")
    parser.add_argument("--all", action="store_true", help="Run all tests")

    # Simulation config
    parser.add_argument("-n", "--num-drones", type=int, default=3, help="Number of drones (default: 3)")
    parser.add_argument("--world", choices=["basic", "complex"], default="complex", help="World type (default: complex)")
    parser.add_argument("--layout", choices=["grid", "line"], default="grid", help="Spawn layout (default: grid)")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument("--skip-sim", action="store_true", help="Assume simulation already running")
    parser.add_argument("--no-ros", action="store_true", help="Skip ROS2 (algorithm testing only)")

    args = parser.parse_args()

    pytest_args = ["-v", "--tb=short"]

    # Unit tests
    if args.unit:
        pytest_args.extend(["-m", "unit", "tests/unit/"])
        return pytest.main(pytest_args)

    # Simulation tests
    if args.sim or args.all:
        if not args.skip_sim:
            sim = SimManager(
                num_drones=args.num_drones,
                world=args.world,
                layout=args.layout,
                headless=args.headless,
                use_ros=not args.no_ros,  # ROS2 is default
            )
            if not sim.start():
                print("Failed to start simulation")
                return 1

        try:
            os.environ["SWARM_SIM_RUNNING"] = "1"
            os.environ["SWARM_NUM_DRONES"] = str(args.num_drones)

            if args.all:
                pytest_args.extend(["tests/"])
            else:
                pytest_args.extend(["-m", "sim", "tests/simulation/"])

            return pytest.main(pytest_args)
        finally:
            if not args.skip_sim:
                sim.stop()

    # Default: unit tests
    pytest_args.extend(["-m", "unit", "tests/unit/"])
    return pytest.main(pytest_args)

if __name__ == "__main__":
    sys.exit(main())
```

---

## World Files

### Basic World (`worlds/basic.sdf`)

Minimal world for algorithm testing:
- Ground plane
- Sun
- No obstacles
- Auto-generated drone models based on `-n`

### Complex World (`worlds/complex.sdf`) - DEFAULT

Full perception test world (renamed from `perception_test.sdf`):
- Ground plane, sun
- People models (for YOLO detection)
- Truck models (for YOLO detection)
- Buildings (urban terrain)

**Note:** GPS denial zones are NOT in the world file. They're configured at runtime via `GPSJammer` class.

### Dynamic Model Generation

Models are generated on-demand for any drone count:

```python
# swarm/simulation/world_generator.py

import math
from pathlib import Path

def get_spawn_positions(num_drones: int, spacing: float = 5.0, layout: str = "grid") -> list[tuple]:
    """Calculate spawn positions for N drones.

    Args:
        num_drones: Number of drones
        spacing: Meters between drones
        layout: "grid" (sqrt x sqrt) or "line"

    Returns:
        List of (x, y, z) positions
    """
    if layout == "line":
        return [(i * spacing, 0.0, 0.0) for i in range(num_drones)]

    # Grid layout: ceil(sqrt(N)) x ceil(sqrt(N))
    cols = math.ceil(math.sqrt(num_drones))
    positions = []
    for i in range(num_drones):
        row, col = divmod(i, cols)
        positions.append((col * spacing, row * spacing, 0.0))
    return positions

def generate_world(num_drones: int, world_type: str = "complex", layout: str = "grid") -> Path:
    """Generate world file with specified number of drones.

    Args:
        num_drones: Number of drone models to include
        world_type: "basic" or "complex"
        layout: "grid" or "line"

    Returns:
        Path to generated world file
    """
    positions = get_spawn_positions(num_drones, layout=layout)

    # Generate DroneN models if they don't exist
    for i in range(1, num_drones + 1):
        model_path = MODELS_DIR / f"Drone{i}"
        if not model_path.exists():
            generate_drone_model(i, positions[i-1])

    # Generate world from template
    template = TEMPLATES_DIR / f"{world_type}.sdf.jinja"
    output = WORLDS_DIR / f"generated_{num_drones}.sdf"

    render_template(template, output, num_drones=num_drones, positions=positions)
    return output
```

### Spawn Layout Examples

**Grid (default) for 6 drones:**
```
D4  D5  D6
D1  D2  D3
```

**Line for 6 drones:**
```
D1  D2  D3  D4  D5  D6
```

---

## SimManager Class

Extracted from duplicated ProcessManager code. Handles full stack: ROS2 + Gazebo + SITL.

```python
# swarm/simulation/sim_manager.py

class SimManager:
    """Manages ROS2 + Gazebo + SITL processes for simulation.

    Default mode starts the full stack including ROS2 nodes.
    Use `use_ros=False` for algorithm testing without ROS2.

    Usage:
        # Full stack (default)
        sim = SimManager(num_drones=6, world="complex")

        if sim.start():
            # Run tests via SwarmBridge (ROS2)
            sim.stop()

        # Or as context manager:
        with SimManager(num_drones=6) as sim:
            # Tests run here...

        # Without ROS2 (algorithm testing only):
        sim = SimManager(num_drones=3, use_ros=False)
    """

    def __init__(
        self,
        num_drones: int = 3,
        world: str = "complex",
        layout: str = "grid",
        headless: bool = False,
        use_ros: bool = True,  # ROS2 is default
        ardupilot_path: Path = None,
    ):
        self.num_drones = num_drones
        self.world = world
        self.layout = layout
        self.headless = headless
        self.use_ros = use_ros
        self.ardupilot_path = self._find_ardupilot(ardupilot_path)

        self._gazebo_proc = None
        self._sitl_procs = []
        self._ros_procs = []
        self._log_dir = PROJECT_ROOT / "logs"

    def start(self, timeout: float = 120.0) -> bool:
        """Start full simulation stack.

        Order:
        1. Generate world with drone models (grid/line layout)
        2. Start Gazebo with generated world
        3. Start SITL instances (one per drone)
        4. Start ROS2 nodes (if use_ros=True)
        5. Wait for EKF convergence

        Returns:
            True if all processes started and ready.
        """
        self._log_dir.mkdir(exist_ok=True)

        # 1. Generate world
        world_path = WorldGenerator.generate(
            self.num_drones,
            self.world,
            self.layout
        )

        # 2. Start Gazebo
        if not self._start_gazebo(world_path):
            return False

        # 3. Start SITL instances
        if not self._start_sitl_instances():
            self.stop()
            return False

        # 4. Start ROS2 nodes (SwarmBridge, VIO, perception)
        if self.use_ros:
            if not self._start_ros_nodes():
                self.stop()
                return False

        # 5. Wait for EKF convergence
        if not self._wait_for_ready(timeout):
            self.stop()
            return False

        return True

    def _start_ros_nodes(self) -> bool:
        """Start ROS2 launch file with SwarmBridge and other nodes."""
        cmd = [
            "ros2", "launch", "swarm_ros", "simulation.launch.py",
            f"num_drones:={self.num_drones}",
        ]
        # ... implementation ...
        return True

    def stop(self) -> None:
        """Stop all simulation processes (reverse order)."""
        # Stop ROS2 nodes
        for proc in self._ros_procs:
            # ...
        # Stop SITL
        for proc in self._sitl_procs:
            # ...
        # Stop Gazebo
        if self._gazebo_proc:
            # ...

    def __enter__(self):
        if not self.start():
            raise RuntimeError("Failed to start simulation")
        return self

    def __exit__(self, *args):
        self.stop()
```

---

## Developer Guide

### Creating New Test Scenarios

#### 1. Unit Test (No Simulation)

```python
# tests/unit/test_my_feature.py

import pytest
from swarm.coordination import MyFeature

class TestMyFeature:
    """Unit tests for MyFeature class."""

    @pytest.mark.unit
    def test_basic_operation(self):
        """Test basic functionality."""
        feature = MyFeature()
        result = feature.compute()
        assert result == expected

    @pytest.mark.unit
    @pytest.mark.parametrize("num_drones", [3, 6, 10])
    def test_scales_with_drones(self, num_drones):
        """Test with different swarm sizes."""
        feature = MyFeature(num_drones=num_drones)
        assert feature.is_valid()
```

#### 2. Integration Test (Requires Simulation)

```python
# tests/integration/test_my_feature.py

import pytest
import os
from swarm.coordination import SwarmController, SwarmConfig

# Check if simulation is running
SIM_RUNNING = os.environ.get("SWARM_SIM_RUNNING") == "1"

class TestMyFeatureIntegration:
    """Integration tests for MyFeature with real simulation."""

    @pytest.fixture
    def controller(self):
        """Create and connect SwarmController."""
        num_drones = int(os.environ.get("SWARM_NUM_DRONES", "3"))
        config = SwarmConfig(num_drones=num_drones)
        ctrl = SwarmController(config)
        ctrl.connect_all()
        ctrl.wait_for_ekf_all()
        yield ctrl
        ctrl.disconnect_all()

    @pytest.mark.integration
    @pytest.mark.skipif(not SIM_RUNNING, reason="Requires simulation")
    def test_feature_in_flight(self, controller):
        """Test feature with actual drones."""
        controller.set_mode_all("GUIDED")
        controller.arm_all()
        controller.takeoff_all(altitude=5.0)

        # Test your feature
        result = your_feature(controller)

        assert result.success
        controller.land_all()
```

#### 3. GPU Test (YOLO)

```python
# tests/unit/test_perception.py

import pytest

class TestYOLODetector:

    @pytest.mark.gpu
    def test_detection_accuracy(self):
        """Test YOLO detection (requires GPU)."""
        from swarm.perception import YOLODetector

        detector = YOLODetector()
        # ... test ...
```

### Running Tests

```bash
# Quick unit tests (< 30 seconds)
python scripts/run_tests.py --unit

# Integration with 3 drones
python scripts/run_tests.py --integration -n 3

# Specific scenario
python scripts/run_tests.py --scenario gps_denied -n 3

# Full test suite with 6 drones
python scripts/run_tests.py --all -n 6

# In Docker
./docker/scripts/run.sh test --integration -n 3
```

### Adding New Scenarios

1. **Unit test first:** Add to `tests/unit/test_*.py`
2. **Add integration test:** Add to `tests/integration/test_*.py`
3. **Use markers:** `@pytest.mark.unit`, `@pytest.mark.integration`, `@pytest.mark.gpu`
4. **Parametrize for scale:** `@pytest.mark.parametrize("num_drones", [3, 6])`
5. **Document in this guide**

### World Customization

To add a new world type:

1. Create template: `worlds/templates/my_world.sdf.jinja`
2. Register in `WorldGenerator.WORLD_TYPES`
3. Use: `python scripts/run_tests.py --world my_world`

---

## Implementation Steps

### Step 1: Create Infrastructure Module

1. [ ] Create `swarm/simulation/__init__.py`
2. [ ] Create `swarm/simulation/sim_manager.py` (extract from `run_phase3_test.py`)
3. [ ] Create `swarm/simulation/world_generator.py` (extract from `generate_world.py`)
   - Add grid spawn layout (sqrt x sqrt)
   - Add line spawn layout
4. [ ] Create `swarm/simulation/sitl_launcher.py` (extract from `launch_sitl.py`)

### Step 2: Create New Test Structure

1. [ ] Create `tests/unit/` directory
2. [ ] Create `tests/simulation/` directory
3. [ ] Create `tests/conftest.py` with shared fixtures and markers
4. [ ] Move unit tests:
   - `test_phase2.py` → `tests/unit/test_config.py` (config tests only)
   - `test_phase3.py` → `tests/unit/test_formations.py`
   - `test_navigation.py` → `tests/unit/test_navigation.py`
   - `test_search_patterns.py` → `tests/unit/test_search_patterns.py`
   - `test_pursuit.py` → `tests/unit/test_pursuit.py`
   - `test_gps_jammer.py` → `tests/unit/test_gps_jammer.py`
5. [ ] Create `tests/simulation/test_swarm.py` (full stack test)

### Step 3: Create Unified Test Runner

1. [ ] Create `scripts/run_tests.py`
   - `--unit` for fast Python tests (GitHub Actions)
   - `--sim` for full stack (ROS2 + Gazebo + SITL)
   - `-n` for drone count
   - `--world` for basic/complex
   - `--layout` for grid/line
   - `--no-ros` edge case flag

### Step 4: Delete Deprecated Code

1. [ ] Delete MAVSDK classes:
   - `swarm/core/fleet.py`
   - `swarm/core/drone.py`
2. [ ] Delete deprecated config classes from `swarm/core/config.py`:
   - `DroneConfig`
   - `FleetConfig`
3. [ ] Update `swarm/core/__init__.py`
4. [ ] Delete deprecated scripts:
   - `scripts/test_connection.py`
   - `scripts/test_offboard.py`
   - `scripts/test_fleet.py`
   - `scripts/test_phase3.py`
   - `scripts/test_phase5.py`
   - `scripts/test_phase6.py`
   - `scripts/test_phase7.py`
   - `scripts/run_phase3_test.py`
   - `scripts/run_integration_tests.py`
5. [ ] Delete old test files:
   - `tests/test_phase1.py`
   - `tests/test_phase2.py`

### Step 5: World File Cleanup

1. [ ] Create `worlds/templates/basic.sdf.jinja`
2. [ ] Rename `worlds/perception_test.sdf` → `worlds/templates/complex.sdf.jinja`
3. [ ] Delete static world files:
   - `worlds/single_drone.sdf`
   - `worlds/multi_drone_3.sdf`
   - `worlds/multi_drone_4.sdf`
   - `worlds/multi_drone_6.sdf`
   - `worlds/terrain_urban_4.sdf`

### Step 6: Documentation Update

1. [ ] Update `CLAUDE.md` with new structure
2. [ ] Update `INSTALL.md` with new test commands
3. [ ] Add developer guide section to this plan (already included below)

---

## Verification Checklist

After refactoring:

- [ ] `python scripts/run_tests.py --unit` passes (also in GitHub Actions)
- [ ] `python scripts/run_tests.py --sim -n 3` passes (full ROS2 stack)
- [ ] `python scripts/run_tests.py --sim -n 6 --world complex` passes
- [ ] `python scripts/run_tests.py --sim -n 3 --no-ros` passes (edge case)
- [ ] `./docker/scripts/run.sh test --sim -n 6` passes
- [ ] All tests have descriptive names (no "phaseN")
- [ ] No MAVSDK imports remain
- [ ] No `Fleet` or `Drone` class references remain
- [ ] Models auto-generate for any drone count (grid layout)
- [ ] Both worlds (basic, complex) work
- [ ] Documentation is updated

---

## Migration Notes

### Command Changes

```bash
# OLD:
python scripts/run_phase3_test.py --num-drones 3
python scripts/test_phase7.py --scenario all
python scripts/run_integration_tests.py

# NEW:
python scripts/run_tests.py --sim -n 3
python scripts/run_tests.py --sim -n 6 --world complex
python scripts/run_tests.py --unit  # for fast tests only
```

### Breaking Changes

1. `Fleet` and `Drone` classes **deleted** - use `SwarmController`
2. `DroneConfig` and `FleetConfig` **deleted** - use `SwarmConfig`
3. All `test_phaseN.py` files renamed or deleted
4. World files generated dynamically - don't commit `worlds/generated_*.sdf`
5. ROS2 is now the **default** for simulation tests
