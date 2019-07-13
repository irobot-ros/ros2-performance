#!/bin/bash

if [ -z "$TARGET_ARCHITECTURE" ]; then
    echo "Missing TARGET_ARCHITECTURE environment variables. Please run first"
    echo "source env.sh";
    exit 1;
fi

if [ -z "$1" ]; then
    THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
    echo "Working on $THIS_DIR workspace"
else
    echo "Moving to $1 workspace"
    cd $1
fi

echo "Ignoring packages..."

# Packages ignored regardless of the TARGET_ARCHITECTURE

touch \
    src/ros2/ros1_bridge/COLCON_IGNORE \
    src/ros2/rviz/COLCON_IGNORE \
    src/ros2/demos/COLCON_IGNORE \
    src/ros/resource_retriever/COLCON_IGNORE \
    src/ros-visualization/COLCON_IGNORE
