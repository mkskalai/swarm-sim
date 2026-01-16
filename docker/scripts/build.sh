#!/bin/bash
# Build the Docker image for Drone Swarm Simulation
#
# Usage:
#   ./build.sh              # Build with default settings
#   ./build.sh --no-cache   # Build without cache (full rebuild)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_DIR="$(dirname "$DOCKER_DIR")"

echo "=== Building Drone Swarm Simulation Docker Image ==="
echo "Project directory: $PROJECT_DIR"
echo ""

cd "$PROJECT_DIR"

# Check if docker compose is available
if command -v docker &> /dev/null && docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
elif command -v docker-compose &> /dev/null; then
    COMPOSE_CMD="docker-compose"
else
    echo "Error: docker compose not found"
    exit 1
fi

# Build with progress output
echo "Building image (this may take 30-45 minutes on first build)..."
echo ""

$COMPOSE_CMD -f docker/docker-compose.yml build --progress=plain "$@"

echo ""
echo "=== Build Complete ==="
echo ""
echo "Image: swarm-simulation:latest"
echo ""
echo "Quick start:"
echo "  # Development mode (with source mounts):"
echo "  cd $DOCKER_DIR && ./scripts/run.sh dev"
echo ""
echo "  # Interactive shell:"
echo "  docker compose -f docker/docker-compose.yml run --rm swarm bash"
echo ""
echo "  # Run tests:"
echo "  docker compose -f docker/docker-compose.yml --profile ci run swarm-ci"
