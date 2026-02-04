#!/bin/bash
# Copyright (c) 2026 Jiayi Hoffman.
#
# Start the LEGO robot Docker container
# Supports both local images and Docker Hub images

set -e  # Exit on error

# Check for --pull flag to force pull from Docker Hub
FORCE_PULL=false
if [[ "$1" == "--pull" ]]; then
    FORCE_PULL=true
    shift
fi

# Image configuration
# Can be overridden by environment variable or command line argument
IMAGE_NAME="${1:-${DOCKER_IMAGE_NAME:-lego_audi_etron}}"

# If DOCKERHUB_USERNAME is set and image doesn't contain a slash, use Docker Hub format
if [ -z "$1" ] && [ -n "$DOCKERHUB_USERNAME" ] && [[ ! "$IMAGE_NAME" =~ / ]]; then
    IMAGE_NAME="${DOCKERHUB_USERNAME}/lego_audi_etron:latest"
fi

echo "=========================================="
echo "Starting LEGO robot container..."
echo "=========================================="
echo ""
echo "Image: ${IMAGE_NAME}"
echo ""

# Force pull if requested
if [ "$FORCE_PULL" = true ]; then
    echo "Pulling latest image from Docker Hub..."
    docker pull "${IMAGE_NAME}"
    echo ""
fi

# Check if image exists locally
if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "Image '${IMAGE_NAME}' not found locally."
    if [ -n "$DOCKERHUB_USERNAME" ] || [[ "$IMAGE_NAME" =~ / ]]; then
        echo "Pulling from Docker Hub..."
        docker pull "${IMAGE_NAME}"
    else
        echo ""
        read -p "Do you want to pull it from Docker Hub? (y/n) " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "Pulling image..."
            docker pull "${IMAGE_NAME}"
        else
            echo "Exiting. Please build or pull the image first."
            exit 1
        fi
    fi
    echo ""
fi

echo "Note: This container requires:"
echo "  - --privileged flag (for Bluetooth/D-Bus access)"
echo "  - --network=host (for ROS2 communication)"
echo "  - D-Bus socket mount (for SimpleBLE)"
echo ""

# Run the container
docker run -it --rm \
    --network=host \
    --privileged \
    -v /var/run/dbus:/var/run/dbus \
    "${IMAGE_NAME}"
