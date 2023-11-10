#! /bin/bash

if [ -z ${PX4_DOCKER_REPO+x} ]; then
	echo "guessing PX4_DOCKER_REPO based on input";
	if [[ $@ =~ .*px4_fmu.* ]]; then
		# nuttx-px4fmu-v{1,2,3,4,5}
		PX4_DOCKER_REPO="px4io/px4-dev-nuttx-focal:2021-09-08"
	elif [[ $@ =~ .*navio2.* ]] || [[ $@ =~ .*raspberry.* ]] || [[ $@ =~ .*beaglebone.* ]] || [[ $@ =~ .*pilotpi.default ]]; then
		# beaglebone_blue_default, emlid_navio2_default, px4_raspberrypi_default, scumaker_pilotpi_default
		PX4_DOCKER_REPO="px4io/px4-dev-armhf:2021-08-18"
	elif [[ $@ =~ .*pilotpi.arm64 ]]; then
		# scumaker_pilotpi_arm64
		PX4_DOCKER_REPO="px4io/px4-dev-aarch64:latest"
	elif [[ $@ =~ .*navio2.* ]] || [[ $@ =~ .*raspberry.* ]] || [[ $@ =~ .*bebop.* ]]; then
		# posix_rpi_cross, posix_bebop_default
		PX4_DOCKER_REPO="px4io/px4-dev-armhf:2021-08-18"
	elif [[ $@ =~ .*clang.* ]] || [[ $@ =~ .*scan-build.* ]]; then
		# clang tools
		PX4_DOCKER_REPO="px4io/px4-dev-clang:2021-02-04"
	elif [[ $@ =~ .*tests* ]]; then
		# run all tests with simulation
		PX4_DOCKER_REPO="px4io/px4-dev-simulation-bionic:2021-12-11"
	fi
else
	echo "PX4_DOCKER_REPO is set to '$PX4_DOCKER_REPO'";
fi

# otherwise default to nuttx
if [ -z ${PX4_DOCKER_REPO+x} ]; then
	PX4_DOCKER_REPO="liuwhale/px4-dev-ros2-humble:cuda12.2.2-cudnn8"
fi

# docker hygiene

#Delete all stopped containers (including data-only containers)
#docker rm $(docker ps -a -q)

#Delete all 'untagged/dangling' (<none>) images
#docker rmi $(docker images -q -f dangling=true)

echo "PX4_DOCKER_REPO: $PX4_DOCKER_REPO";

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../../

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"
## common use
# docker run -it --rm -w "${SRC_DIR}" \
# 	--env=AWS_ACCESS_KEY_ID \
# 	--env=AWS_SECRET_ACCESS_KEY \
# 	--env=BRANCH_NAME \
# 	--env=CCACHE_DIR="${CCACHE_DIR}" \
# 	--env=CI \
# 	--env=CODECOV_TOKEN \
# 	--env=COVERALLS_REPO_TOKEN \
# 	--env=LOCAL_USER_ID="$(id -u)" \
# 	--env=PX4_ASAN \
# 	--env=PX4_MSAN \
# 	--env=PX4_TSAN \
# 	--gpus all \
# 	-v /tmp/.x11-unix:/tmp/.x11-unix \
# 	-e DISPLAY=10.147.18.222:0.0 \
# 	-e QT_DEBUG_PLUGINS=1 \
# 	--shm-size=1g \
# 	--ulimit memlock=-1 \
# 	--ulimit stack=67108864 \
# 	--env=PX4_UBSAN \
# 	--env=TRAVIS_BRANCH \
# 	--env=TRAVIS_BUILD_ID \
# 	--publish 14556:14556/udp \
# 	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
# 	--volume=${SRC_DIR}:${SRC_DIR}:rw \
# 	${PX4_DOCKER_REPO} /bin/bash -c "$1 $2 $3"

## proxy commands
# -e http_proxy=http://10.147.18.222:7890 \
# -e https_proxy=http://10.147.18.222:7890 \
# --publish 7890:7890 \

## install nvidia driver in container
# apt install kmod -y
# ./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
# __GL_SYNC_TO_VBLANK=0 glxgears

## mostly for wsl
## if you wanna use rootless mode, you need to edit /etc/nvidia-container-runtime/config.toml
## [nvidia-container-cli]
## no-cgroups = true
## but it'll lead to an issue: Failed to initialize NVML: Unknown Error with nvidia-smi

sudo rocker --nvidia --x11 \
	--home \
	--env=AWS_ACCESS_KEY_ID \
	--env=AWS_SECRET_ACCESS_KEY \
	--env=BRANCH_NAME \
	--env=CCACHE_DIR="${CCACHE_DIR}" \
	--env=CI \
	--env=CODECOV_TOKEN \
	--env=COVERALLS_REPO_TOKEN \
	--env=PX4_ASAN \
	--env=PX4_MSAN \
	--env=PX4_TSAN \
	--env=PX4_UBSAN \
	--env=TRAVIS_BRANCH \
	--env=TRAVIS_BUILD_ID \
	--env=GZ_IP=10.147.18.157 \
	--env=GZ_VERBOSE=0 \
	--net=host \
	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
	--volume=${SRC_DIR}:${SRC_DIR}:rw \
	${PX4_DOCKER_REPO}

#########################################
##			               ##
##    If gz's client and server are    ##
##    naturally in same pc or wlan,    ##
##    dont need to set env like        ##
##    GZ_IP, --net=host or             ##
##    GZ_SIM_RESOURCE_PATH. 	       ##
#########################################
##				       ##
##	   Recommend zerotier.	       ##
##				       ##
#########################################

## It can make container use host IPs and ports
#  --net=host

## gazebo client run on host, not on docker
## (make sure container can ping host ip)
## execute commands below on host
#  export GZ_PARTITION=Cetacea:user

## Cetacea is hostname, user is username in container
## this env variable is for resource path
#  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
#  $HOME/humble_src/PX4-Autopilot/Tools/simulation/gz/models

## this env vari make sure client and server in same route
#  export GZ_IP=10.147.18.221\157

## this env is for debug switch
#  export  GZ_VERBOSE=1

## Other commands
#  gz sim -v 4 -g
#  HEADLESS=1 make px4_sitl gz_x500

##Troubleshooting
## If gz shows the color in white or glxgears in black
##work-around Methods
# 1. GALLIUM_DRIVER=llvmpipe glxgears
# 2. LIBGL_ALWAYS_SOFTWARE=1 glxgears
