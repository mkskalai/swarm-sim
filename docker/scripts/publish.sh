#!/bin/bash
# Publish Docker image to a container registry
#
# Usage:
#   ./publish.sh dockerhub USERNAME          # Push to Docker Hub
#   ./publish.sh ghcr USERNAME               # Push to GitHub Container Registry
#   ./publish.sh custom registry.example.com # Push to custom registry

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="swarm-simulation"
IMAGE_TAG="latest"
SOURCE_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

usage() {
    echo "Usage: $0 <registry> <username/registry-url>"
    echo ""
    echo "Registries:"
    echo "  dockerhub <username>    Push to Docker Hub"
    echo "  ghcr <username>         Push to GitHub Container Registry"
    echo "  custom <registry-url>   Push to custom registry"
    echo ""
    echo "Examples:"
    echo "  $0 dockerhub myuser"
    echo "  $0 ghcr myuser"
    echo "  $0 custom registry.example.com/myproject"
    exit 1
}

if [ $# -lt 2 ]; then
    usage
fi

REGISTRY="$1"
TARGET="$2"

# Check source image exists
if ! docker image inspect "$SOURCE_IMAGE" &> /dev/null; then
    echo "Error: Source image '$SOURCE_IMAGE' not found"
    echo "Run './build.sh' first to build the image"
    exit 1
fi

case "$REGISTRY" in
    dockerhub)
        REMOTE_IMAGE="docker.io/${TARGET}/${IMAGE_NAME}:${IMAGE_TAG}"
        echo "=== Publishing to Docker Hub ==="
        echo "Logging in to Docker Hub..."
        docker login
        ;;
    ghcr)
        REMOTE_IMAGE="ghcr.io/${TARGET}/${IMAGE_NAME}:${IMAGE_TAG}"
        echo "=== Publishing to GitHub Container Registry ==="
        echo "Logging in to ghcr.io..."
        echo "Note: Use a GitHub Personal Access Token with 'write:packages' scope"
        docker login ghcr.io -u "$TARGET"
        ;;
    custom)
        REMOTE_IMAGE="${TARGET}/${IMAGE_NAME}:${IMAGE_TAG}"
        echo "=== Publishing to ${TARGET} ==="
        echo "Logging in to ${TARGET}..."
        docker login "${TARGET%%/*}"
        ;;
    *)
        echo "Error: Unknown registry '$REGISTRY'"
        usage
        ;;
esac

echo ""
echo "Tagging: $SOURCE_IMAGE -> $REMOTE_IMAGE"
docker tag "$SOURCE_IMAGE" "$REMOTE_IMAGE"

echo "Pushing: $REMOTE_IMAGE"
docker push "$REMOTE_IMAGE"

echo ""
echo "=== Published Successfully ==="
echo "Image: $REMOTE_IMAGE"
echo ""
echo "To pull:"
echo "  docker pull $REMOTE_IMAGE"
