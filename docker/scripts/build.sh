#!/bin/bash
# Build the Docker image for Drone Swarm Simulation
#
# Usage:
#   ./build.sh              # Build full image (~15-20GB)
#   ./build.sh minimal      # Build minimal image, no perception stack (~8-12GB)
#   ./build.sh --no-cache   # Build without cache (full rebuild)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_DIR="$(dirname "$DOCKER_DIR")"

# Check for minimal build
MINIMAL="false"
BUILD_ARGS=""
IMAGE_TAG="swarm-simulation:latest"

if [ "$1" = "minimal" ]; then
    MINIMAL="true"
    IMAGE_TAG="swarm-simulation:minimal"
    shift
fi

echo "=== Building Drone Swarm Simulation Docker Image ==="
echo "Project directory: $PROJECT_DIR"
echo "Build type: $([ "$MINIMAL" = "true" ] && echo "MINIMAL (no perception)" || echo "FULL")"
echo "Image tag: $IMAGE_TAG"
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

if [ "$MINIMAL" = "true" ]; then
    # Build minimal image directly
    docker build -t "$IMAGE_TAG" --build-arg MINIMAL=true --progress=plain -f docker/Dockerfile "$@" .
else
    # Build via compose for full image
    $COMPOSE_CMD -f docker/docker-compose.yml build --progress=plain "$@"
fi

echo ""
echo "=== Build Complete ==="
echo ""
echo "Image: $IMAGE_TAG"
if [ "$MINIMAL" = "true" ]; then
    echo "Note: Minimal build - YOLO/PyTorch/OpenCV/GStreamer not available"
fi
echo ""
echo "Quick start:"
echo "  # Development mode (with source mounts):"
echo "  cd $DOCKER_DIR && ./scripts/run.sh dev"
echo ""
echo "  # Interactive shell:"
if [ "$MINIMAL" = "true" ]; then
    echo "  docker run --rm -it --network host $IMAGE_TAG bash"
else
    echo "  docker compose -f docker/docker-compose.yml run --rm swarm bash"
fi
echo ""
echo "  # Run tests:"
echo "  docker compose -f docker/docker-compose.yml --profile ci run swarm-ci"
