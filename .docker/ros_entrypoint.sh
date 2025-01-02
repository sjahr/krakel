#!/bin/bash

# Setup ROS 2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "${WORKSPACE}/install/setup.bash" --

# Ensure pre-commit can be run from inside the container
git config --global --add safe.directory "${WORKSPACE}/src/krakel"

# Setup Pinocchio environment
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# Execute docker command
exec "$@"
