#!/bin/bash

ROS2_PATH=~/ros2_cc_ws

# You can specify a distribution, otherwise the default one will be used
distribution="eloquent"
if [ -z "$1" ]; then
  echo "Using default ROS2 $distribution distribution sources"
else
  distribution=$1
  echo "Using ROS2 $distribution distribution sources"
fi

if [ -d $ROS2_PATH ]; then
    echo "Error: Directory $ROS2_PATH already exists, remove it to get new sources."
    exit 1
else
    # Download sources for distribution
    echo "Downloading ROS2 sources in directory: $ROS2_PATH"
    mkdir -p $ROS2_PATH/src
    cd $ROS2_PATH
    wget https://raw.githubusercontent.com/ros2/ros2/$distribution/ros2.repos
    vcs import src < ros2.repos
fi
