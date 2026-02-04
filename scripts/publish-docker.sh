#!/bin/bash
# Copyright (c) 2026 Jiayi Hoffman.
#
# Build and publish Docker image to Docker Hub

set -e  # Exit on error

# Check if DOCKERHUB_USERNAME is set
if [ -z "$DOCKERHUB_USERNAME" ]; then
    echo "Error: DOCKERHUB_USERNAME environment variable is not set"
    echo "Please set it with: export DOCKERHUB_USERNAME=your_username"
    exit 1
fi

# Image configuration
IMAGE_NAME="lego_audi_etron"
IMAGE_TAG="latest"
FULL_IMAGE_NAME="${DOCKERHUB_USERNAME}/${IMAGE_NAME}:${IMAGE_TAG}"

# Get the directory where this script is located and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKERFILE_DIR="${PROJECT_ROOT}/docker"

echo "=========================================="
echo "Building Docker image: ${FULL_IMAGE_NAME}"
echo "=========================================="

# Build the Docker image
# Build context is the project root, Dockerfile is in docker/ subdirectory
docker build \
    -t "${FULL_IMAGE_NAME}" \
    -f "${DOCKERFILE_DIR}/Dockerfile" \
    "${PROJECT_ROOT}"

echo ""
echo "=========================================="
echo "Build successful!"
echo "=========================================="
echo ""
echo "Image: ${FULL_IMAGE_NAME}"
echo ""
read -p "Do you want to push this image to Docker Hub? (y/n) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "=========================================="
    echo "Pushing to Docker Hub..."
    echo "=========================================="
    
    # Login check (will prompt if not already logged in)
    if ! docker info | grep -q "Username"; then
        echo "Please login to Docker Hub:"
        docker login
    fi
    
    # Push the image
    docker push "${FULL_IMAGE_NAME}"
    
    echo ""
    echo "=========================================="
    echo "Push successful!"
    echo "=========================================="
    echo ""
    echo "Image is now available at: docker.io/${FULL_IMAGE_NAME}"
else
    echo "Skipping push. Image is available locally as: ${FULL_IMAGE_NAME}"
fi
