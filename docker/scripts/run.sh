#!/bin/bash
# Run the Drone Swarm Simulation Docker container
#
# Usage:
#   ./run.sh              # Interactive shell (default)
#   ./run.sh dev          # Development mode with source mounts
#   ./run.sh gpu          # Development mode with GPU (requires nvidia-container-toolkit)
#   ./run.sh ci           # CI mode - run unit tests (headless, no GPU)
#   ./run.sh test [path]  # Run specific tests (no GPU)
#   ./run.sh test-gpu [path]  # Run specific tests with GPU
#   ./run.sh integration  # Run integration tests (requires SITL+Gazebo, no GPU)
#   ./run.sh integration-gpu  # Run integration tests with GPU
#   ./run.sh sim-test [N]     # Start sim + run integration tests (default 3 drones)
#   ./run.sh sim-test-gpu [N] # Start sim + run integration tests with GPU
#   ./run.sh sim-test-headless [N]  # Headless sim + tests
#   ./run.sh headless     # Headless simulation
#   ./run.sh gazebo       # Launch Gazebo GUI only
#   ./run.sh sim N        # Run N-drone simulation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_DIR="$(dirname "$DOCKER_DIR")"

cd "$PROJECT_DIR"

# Setup X11 forwarding for GUI
setup_x11() {
    if [ -n "$DISPLAY" ]; then
        echo "[run.sh] Setting up X11 forwarding..."
        # Allow local docker connections to X server
        xhost +local:docker 2>/dev/null || true
    fi
}

# Check if docker compose is available
if command -v docker &> /dev/null && docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
elif command -v docker-compose &> /dev/null; then
    COMPOSE_CMD="docker-compose"
else
    echo "Error: docker compose not found"
    exit 1
fi

COMPOSE_FILE="docker/docker-compose.yml"

case "${1:-shell}" in
    dev)
        echo "=== Development Mode ==="
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE --profile dev run --rm swarm-dev bash
        ;;

    gpu)
        echo "=== GPU Development Mode (requires nvidia-container-toolkit) ==="
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE --profile gpu run --rm swarm-gpu bash
        ;;

    ci)
        echo "=== CI Mode - Running Tests ==="
        $COMPOSE_CMD -f $COMPOSE_FILE --profile ci run --rm swarm-ci
        ;;

    headless)
        echo "=== Headless Simulation ==="
        $COMPOSE_CMD -f $COMPOSE_FILE --profile headless run --rm swarm-headless
        ;;

    gazebo)
        echo "=== Gazebo GUI ==="
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE run --rm swarm gz sim -r /app/worlds/multi_drone_3.sdf
        ;;

    sim)
        NUM_DRONES="${2:-3}"
        echo "=== Running $NUM_DRONES-drone Simulation ==="
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE --profile dev run --rm swarm-dev \
            python3 scripts/run_phase3_test.py --num-drones "$NUM_DRONES" --skip-test
        ;;

    test)
        TEST_FILE="${2:-tests/}"
        echo "=== Running Tests: $TEST_FILE (CPU mode) ==="
        $COMPOSE_CMD -f $COMPOSE_FILE run --rm swarm pytest "$TEST_FILE" -v
        ;;

    test-gpu)
        TEST_FILE="${2:-tests/}"
        echo "=== Running Tests: $TEST_FILE (GPU mode) ==="
        $COMPOSE_CMD -f $COMPOSE_FILE --profile gpu run --rm swarm-gpu pytest "$TEST_FILE" -v
        ;;

    integration)
        echo "=== Running Integration Tests (CPU mode) ==="
        echo "Note: Integration tests require SITL+Gazebo simulation stack"
        setup_x11
        # Run tests marked as integration (skip the @pytest.mark.skip decorator)
        $COMPOSE_CMD -f $COMPOSE_FILE --profile dev run --rm swarm-dev \
            pytest tests/ -v -m "integration or asyncio" --ignore=tests/test_phase1.py
        ;;

    integration-gpu)
        echo "=== Running Integration Tests (GPU mode) ==="
        echo "Note: Integration tests require SITL+Gazebo simulation stack"
        setup_x11
        # Run tests marked as integration with GPU acceleration
        $COMPOSE_CMD -f $COMPOSE_FILE --profile gpu run --rm swarm-gpu \
            pytest tests/ -v -m "integration or asyncio" --ignore=tests/test_phase1.py
        ;;

    sim-test)
        NUM_DRONES="${2:-3}"
        echo "=== Running Integration Tests with Simulation ($NUM_DRONES drones) ==="
        echo "This will start Gazebo + SITL, run tests, then clean up"
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE --profile dev run --rm swarm-dev \
            python3 scripts/run_integration_tests.py --num-drones "$NUM_DRONES"
        ;;

    sim-test-gpu)
        NUM_DRONES="${2:-3}"
        echo "=== Running Integration Tests with Simulation + GPU ($NUM_DRONES drones) ==="
        echo "This will start Gazebo + SITL, run tests, then clean up"
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE --profile gpu run --rm swarm-gpu \
            python3 scripts/run_integration_tests.py --num-drones "$NUM_DRONES"
        ;;

    sim-test-headless)
        NUM_DRONES="${2:-3}"
        echo "=== Running Integration Tests with Simulation (Headless, $NUM_DRONES drones) ==="
        echo "This will start Gazebo + SITL headless, run tests, then clean up"
        HEADLESS=1 $COMPOSE_CMD -f $COMPOSE_FILE --profile headless run --rm \
            -e HEADLESS=1 swarm-headless \
            python3 scripts/run_integration_tests.py --num-drones "$NUM_DRONES"
        ;;

    shell|bash|"")
        echo "=== Interactive Shell ==="
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE run --rm swarm bash
        ;;

    *)
        # Pass through any other command
        setup_x11
        $COMPOSE_CMD -f $COMPOSE_FILE run --rm swarm "$@"
        ;;
esac
