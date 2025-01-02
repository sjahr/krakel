#!/bin/bash

# Setup ROS 2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "${WORKSPACE}/install/setup.bash" --

# Ensure pre-commit can be run from inside the container
git config --global --add safe.directory "${WORKSPACE}/src/krakel"

# Execute docker command
exec "$@"
