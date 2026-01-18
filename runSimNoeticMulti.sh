#!/bin/bash
# Script to start PX4 multi-drone simulation in Docker with a single command
# This script starts the Docker container and automatically runs the simulation
# based on positions defined in positions.txt

# Get script directory to make paths invariant to project location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONTAINER_NAME="px4-noetic-sim-ros"

# Setup X11 forwarding
xhost +
xhost +local:docker

# Remove existing container if it exists
docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

# Run docker container with the simulation command
docker run -it --net=host \
           --cap-drop=all \
           --privileged \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           --volume="${SCRIPT_DIR}/multidrone/positions.txt:/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt:rw" \
           --volume="${SCRIPT_DIR}/multidrone/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw" \
           --name=${CONTAINER_NAME} \
           ${CONTAINER_NAME} \
           /bin/bash -c "./Tools/simulation/gazebo-classic/sitl_multiple_run2.sh ./Tools/simulation/positions.txt"

# Cleanup X11 forwarding
xhost -local:docker
xhost -
