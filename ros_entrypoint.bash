#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.sh
source $ROS_WS/install/local_setup.sh

# Execute CMD
exec "$@"