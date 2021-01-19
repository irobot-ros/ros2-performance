#!/bin/bash

if [[ $# -ne 2 ]]; then
    echo "Illegal number of parameters. Required to specify distribution and path."
    exit 1
fi

DISTRIBUTION="$1"
WORKSPACE_DIR_PATH="$2"

cd $WORKSPACE_DIR_PATH

# Download sources
wget https://raw.githubusercontent.com/ros2/ros2/$DISTRIBUTION/ros2.repos
vcs import src < ros2.repos

# Ignore some packages
touch \
    src/ros2/ros1_bridge/COLCON_IGNORE \
    src/ros2/rviz/COLCON_IGNORE \
    src/ros2/demos/COLCON_IGNORE \
    src/ros/resource_retriever/COLCON_IGNORE \
    src/ros/robot_state_publisher/COLCON_IGNORE \
    src/ros-visualization/COLCON_IGNORE
