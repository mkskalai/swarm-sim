# Contributing Guide

This guide covers development workflow, testing, and code conventions.

## Development Setup

1. Complete installation per [INSTALL.md](../INSTALL.md)
2. Activate environment in every terminal:
   ```bash
   source ~/setup_swarm_env.sh
   ```

## Project Structure

```
swarm/
├── swarm/              # Main Python package
│   ├── core/           # Drone control, config
│   ├── coordination/   # Formations, missions, swarm control
│   └── perception/     # Detection, tracking
├── swarm_ros/          # ROS2 package
│   ├── msg/, srv/      # Message/service definitions
│   ├── swarm_ros/      # Python nodes
│   └── launch/         # Launch files
├── models/             # Gazebo drone models
├── worlds/             # Gazebo world files
├── scripts/            # Test and utility scripts
└── tests/              # Unit tests
```

---

## Development Workflow

### Making Changes to Python Code (`swarm/`)

Changes take effect immediately (no build needed):

```bash
# Test your changes
python scripts/test_phase3.py --num-drones 3
```

### Making Changes to ROS2 Code (`swarm_ros/`)

Rebuild after changes:

```bash
cd ~/ros2_ws
colcon build --packages-select swarm_ros
source install/setup.bash
```

### Adding New ROS2 Messages/Services

1. Create `.msg` or `.srv` file in `swarm_ros/msg/` or `swarm_ros/srv/`
2. Add to `CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/YourNewMessage.msg"
     ...
   )
   ```
3. Rebuild:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select swarm_ros
   ```

### Adding New Drone Models

Use the generator:
```bash
python scripts/generate_world.py --num-drones N
```

Or manually:
1. Copy `models/DroneTemplate/` to `models/DroneN/`
2. Update `model.config` with new name
3. Update `model.sdf`:
   - Model name
   - `fdm_port_in` (9002 + (N-1)*10)
   - Camera topic (`drone_N-1/camera`)

---

## Testing

### Unit Tests

```bash
# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_phase3.py -v

# Run with coverage
pytest tests/ --cov=swarm --cov-report=html
```

### Integration Tests

```bash
# Full simulation test (starts Gazebo + SITL)
python scripts/run_phase3_test.py --num-drones 3

# Specific test
python scripts/run_phase3_test.py --test formations
python scripts/run_phase3_test.py --test leader_follower
python scripts/run_phase3_test.py --test missions
```

### Manual Testing

```bash
# Start simulation only
python scripts/run_phase3_test.py --num-drones 3 --skip-test

# In another terminal, run your test code
python your_test_script.py
```

---

## Code Conventions

### Python Style

- Follow PEP 8
- Use type hints for function signatures
- Use dataclasses for configuration
- Prefer async/await for I/O operations

```python
from dataclasses import dataclass
from typing import Optional

@dataclass
class MyConfig:
    value: float = 10.0
    name: str = "default"

async def my_function(config: Optional[MyConfig] = None) -> bool:
    """Short description.

    Args:
        config: Configuration options.

    Returns:
        True on success.
    """
    if config is None:
        config = MyConfig()
    # ...
    return True
```

### Logging

Use Python logging, not print:

```python
import logging

logger = logging.getLogger(__name__)

logger.info("Connected to drone %d", drone_id)
logger.warning("Low battery: %.1f%%", battery)
logger.error("Connection failed: %s", error)
```

### Error Handling

- Catch specific exceptions
- Log errors with context
- Return meaningful error states

```python
try:
    await drone.connect()
except ConnectionError as e:
    logger.error("Failed to connect to drone %d: %s", drone_id, e)
    return False
```

---

## Common Development Tasks

### Adding a New Formation

1. Add to `FormationType` enum in `swarm/coordination/formations.py`:
   ```python
   class FormationType(Enum):
       # ... existing ...
       MY_FORMATION = "my_formation"
   ```

2. Implement calculation function:
   ```python
   def my_formation(num_drones: int, config: FormationConfig) -> list[tuple]:
       positions = []
       for i in range(num_drones):
           n = ...  # Calculate north
           e = ...  # Calculate east
           d = -config.altitude
           positions.append((n, e, d))
       return positions
   ```

3. Add to `calculate_formation_positions()`:
   ```python
   elif formation_type == FormationType.MY_FORMATION:
       return my_formation(num_drones, config)
   ```

4. Update ROS2 service mapping in `swarm_bridge.py` if needed

### Adding a New ROS2 Service

1. Create `srv/MyService.srv`:
   ```
   # Request
   int32 param
   ---
   # Response
   bool success
   string message
   ```

2. Add to `CMakeLists.txt`

3. Implement handler in `swarm_bridge.py`:
   ```python
   def __init__(self):
       # ...
       self.my_service = self.create_service(
           MyService, '/swarm/my_service', self._handle_my_service)

   def _handle_my_service(self, request, response):
       # Implementation
       response.success = True
       return response
   ```

4. Rebuild ROS2 workspace

### Adding a New Detection Class

1. Update `DetectorConfig.target_classes` default:
   ```python
   target_classes: list[int] = field(default_factory=lambda: [0, 2, 3, 5, 7, NEW_CLASS])
   ```

2. Or configure at runtime:
   ```python
   config = DetectorConfig(target_classes=[0, 2, 3, 5, 7, 9])  # Add class 9
   ```

---

## Debugging Tips

### SITL Not Connecting

Check logs:
```bash
cat logs/sitl_0_stdout.log
cat logs/sitl_0_stderr.log
```

### Gazebo Issues

Run with verbose output:
```bash
gz sim -v4 -r worlds/multi_drone_3.sdf
```

### ROS2 Node Issues

Check if node is running:
```bash
ros2 node list
ros2 topic list
ros2 service list
```

Check topic data:
```bash
ros2 topic echo /swarm/status
ros2 topic hz /drone_0/state
```

### Python Import Issues

Ensure PYTHONPATH is set:
```bash
echo $PYTHONPATH
# Should include: /path/to/swarm
```

---

## Git Workflow

### Branches

- `main` - Stable, tested code
- `feature/*` - New features
- `fix/*` - Bug fixes

### Commits

Use conventional commits:
```
feat: add diamond formation pattern
fix: correct EKF wait timeout handling
docs: update API reference for SwarmController
refactor: simplify formation calculation
test: add integration test for leader-follower
```

### Before Committing

1. Run tests:
   ```bash
   pytest tests/ -v
   ```

2. Test with simulation:
   ```bash
   python scripts/run_phase3_test.py --num-drones 3
   ```

3. Check ROS2 build:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select swarm_ros
   ```
