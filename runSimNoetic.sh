#!/bin/bash
export PX4_SITL_DOCKER_NAME=px4-noetic-sim-ros
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:latest
# kill all containers
docker kill $PX4_SITL_DOCKER_NAME

./run_px4_sitl_docker.sh 'make px4_sitl gazebo-classic'