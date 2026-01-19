#!/bin/bash
# Script to start PX4 multi-drone simulation in Docker with a single command
# This script starts the Docker container and automatically runs the simulation
# based on positions defined in positions.txt

# Get script directory to make paths invariant to project location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONTAINER_NAME="px4-noetic-sim-ros"

# Cleanup function to be called on script exit
cleanup_on_exit() {
    echo ""
    echo "Cleaning up on exit..."
    docker rm -f ${CONTAINER_NAME} 2>/dev/null || true
    # Clean up X11
    xhost -local:docker 2>/dev/null || true
    xhost - 2>/dev/null || true
    # Clean up any leftover X11 lock files (but not socket files)
    rm -f /tmp/.X*-lock 2>/dev/null || true
}

# Set trap to cleanup on script exit
trap cleanup_on_exit EXIT INT TERM

# Clean up any leftover X11 lock files from previous runs (but not socket files)
echo "Cleaning up X11 resources..."
rm -f /tmp/.X*-lock 2>/dev/null || true

# Setup X11 forwarding
xhost + 2>/dev/null || true
xhost +local:docker 2>/dev/null || true

# Verify display is accessible
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable is not set"
    export DISPLAY=:0
fi

# Set XAUTHORITY if not set
if [ -z "$XAUTHORITY" ] && [ -f "$HOME/.Xauthority" ]; then
    export XAUTHORITY="$HOME/.Xauthority"
fi

# Remove existing container if it exists (whether running or stopped)
echo "Removing existing container if present..."
docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

# Wait a moment to ensure container is fully removed
sleep 1

# Verify container is removed
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Warning: Container still exists, forcing removal..."
    docker rm -f ${CONTAINER_NAME} 2>/dev/null || true
    sleep 1
fi

# Clean up any leftover processes from previous runs
# These might still be running if the previous container crashed
echo "Cleaning up leftover processes..."
pkill -x px4 2>/dev/null || true
pkill gzclient 2>/dev/null || true
pkill gzserver 2>/dev/null || true
# Kill Gazebo master process (runs on port 11345)
pkill -f "gz master" 2>/dev/null || true
pkill -f "gazebo.*master" 2>/dev/null || true
# Give processes a moment to terminate
sleep 2

# Additional cleanup: kill any processes using Gazebo ports
# Gazebo master typically uses port 11345
if lsof -ti:11345 >/dev/null 2>&1; then
    echo "Killing process using Gazebo master port 11345..."
    lsof -ti:11345 | xargs kill -9 2>/dev/null || true
    sleep 1
fi

# Build docker run command with conditional XAUTHORITY volume
XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
DOCKER_VOLUMES=(
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
    --volume="${SCRIPT_DIR}/multidrone/positions.txt:/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt:rw"
    --volume="${SCRIPT_DIR}/multidrone/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw"
)

# Add XAUTHORITY volume only if file exists
if [ -f "$XAUTH_FILE" ]; then
    DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
fi

# Run docker container with the simulation command
docker run -it --net=host \
           --cap-drop=all \
           --privileged \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --env="XAUTHORITY=${XAUTH_FILE}" \
           "${DOCKER_VOLUMES[@]}" \
           --name=${CONTAINER_NAME} \
           ${CONTAINER_NAME} \
           /bin/bash -c "./Tools/simulation/gazebo-classic/sitl_multiple_run2.sh ./Tools/simulation/positions.txt"

# Note: Cleanup is handled by the trap function on exit
