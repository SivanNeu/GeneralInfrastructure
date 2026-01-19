#!/bin/bash
export PX4_SITL_DOCKER_NAME=px4-noetic-sim-ros
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:latest

# Remove existing container if it exists (whether running or stopped)
docker rm -f $PX4_SITL_DOCKER_NAME 2>/dev/null || true

./run_px4_sitl_docker.sh 'make px4_sitl gazebo-classic'