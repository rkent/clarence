#!/bin/bash

# Adapted from
# https://github.com/osrf/docker_images/blob/master/ros2/nightly/nightly/ros_entrypoint.sh
set -e

# setup ros2 environment
if [ -f install/setup.bash ]; then
  echo "Sourcing previously built local workspace"
  source "install/setup.bash" --
else
  echo "Sourcing ROS from /opt"
  source "/opt/ros/$ROS_DISTRO/setup.bash" --
fi
exec "$@"
