#!/bin/bash
# Parameter #1: base variant to load.
DISTRO=${1:-rolling}
VARIANT="${2:-ros-base}"

RANDOM_NUM=$(( (RANDOM) + 1 ))
HOSTNAME=${DISTRO}-${VARIANT}-${RANDOM_NUM}
docker run -it --rm --hostname ${HOSTNAME} --name ${HOSTNAME} \
  --volume ./:/workspace \
  --device /dev/gpiochip0 \
  ros-local:${DISTRO}-${VARIANT}