#!/bin/bash

SCRIPT_DIR=`dirname $( readlink -m $( type -p $0 ))`

# Parameter #1: base variant to load.
DISTRO="${1:-rolling}"
VARIANT="${2:-ros-base}"
cp ~/.gitconfig $SCRIPT_DIR/.gitconfig
docker build -f $SCRIPT_DIR/image/Dockerfile \
  --build-arg DISTRO=${DISTRO} \
  --build-arg USERNAME=$(whoami) \
  --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
  --build-arg GPIO_ID=$(getent group gpio | cut -d: -f3) \
  --build-arg VARIANT=${VARIANT} -t ros-local:${DISTRO}-${VARIANT} \
  --progress=plain \
  $SCRIPT_DIR/..
