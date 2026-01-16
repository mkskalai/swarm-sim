#!/bin/bash
# Run the Drone Swarm Simulation Docker container
#
# Usage:
#   ./run.sh              # Interactive shell (default)
#   ./run.sh dev          # Development mode with source mounts
#   ./run.sh ci           # CI mode - run tests
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
            python scripts/run_phase3_test.py --num-drones "$NUM_DRONES" --skip-test
        ;;

    test)
        TEST_FILE="${2:-tests/}"
        echo "=== Running Tests: $TEST_FILE ==="
        $COMPOSE_CMD -f $COMPOSE_FILE run --rm swarm pytest "$TEST_FILE" -v
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
