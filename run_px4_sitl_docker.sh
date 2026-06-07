#!/bin/bash

# Set default values if not already set
export PX4_SITL_DOCKER_NAME=${PX4_SITL_DOCKER_NAME:-px4-noetic-sim-ros}
export PX4_SITL_DOCKER_VER=${PX4_SITL_DOCKER_VER:-${PX4_SITL_DOCKER_NAME}:latest}
HOST_PX4_PATH='/home/user/catkin_ws/cat'
CONT_PX4_PATH='/home/valentin/PX4-Autopilot'

# enable access to xhost from the container
sudo xhost +

# Set up environment variables for Gazebo Sim
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# GZ_SIM_RESOURCE_PATH is needed for Gazebo Sim to find resources
# This should point to the Gazebo Sim resource directories
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-/usr/share/gz}

docker run -it --rm \
	--gpus all \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-e DISPLAY=$DISPLAY \
	-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
	-e GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH \
	-e QT_X11_NO_MITSHM=1 \
	--network host \
	--privileged \
	--user $(id -u):$(id -g) \
	--name ${PX4_SITL_DOCKER_NAME} \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v /dev/dri:/dev/dri \
	-v $HOME/.Xauthority:/root/.Xauthority:rw \
	-v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
	-v $HOST_PX4_PATH:$CONT_PX4_PATH \
	-w $CONT_PX4_PATH \
	${PX4_SITL_DOCKER_VER} /bin/bash -c "$1 $2 $3"
