#!/bin/bash

# Set default values if not already set
export PX4_SITL_DOCKER_NAME=${PX4_SITL_DOCKER_NAME:-px4_sitl}
export PX4_SITL_DOCKER_VER=${PX4_SITL_DOCKER_VER:-${PX4_SITL_DOCKER_NAME}:v0.1}

# enable access to xhost from the container
sudo xhost +




docker run -it --rm \
	-e DISPLAY=$DISPLAY \
	--network host \
	--privileged \
	--name ${PX4_SITL_DOCKER_NAME} \
	${PX4_SITL_DOCKER_VER} /bin/bash -c "$1 $2 $3"
	# -v $(pwd)/10015_gazebo-classic_iris:/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris \
	# -v $(pwd)/iris.sdf.jinja:/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja \


# PX4_SITL_DOCKER_VER="px4_sitl:v0.1"
# # PX4_SITL_DOCKER_VER="px4-noetic-sim-ros:latest"

# docker run -it --rm \
# 	-e DISPLAY=$DISPLAY \
# 	--network host \
# 	--privileged \
# 	-v /home/$USER/Projects/GambitonBiut/docker_sim/PX4-Autopilot/:/home/$USER/PX4-Autopilot \
# 	-w /home/$USER/PX4-Autopilot \
# 	${PX4_SITL_DOCKER_VER} /bin/bash -c "$1 $2 $3"	
#make px4_sitl none_iris
