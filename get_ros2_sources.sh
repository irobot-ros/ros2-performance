#!/bin/bash

# You can specify a distribution with --distro=<distribution>, otherwise the default one will be used
DISTRIBUTION="eloquent"

# You can specify a directory to store sources with --ros2-path=<DIR>, otherwise the default one will be used
ROS2_PATH=~/ros2_cc_ws

for i in "$@"
do
case $i in
    --distro=*)
    DISTRIBUTION="${i#*=}"
    shift
    ;;
    --ros2-path=*)
    ROS2_PATH="${i#*=}"
    shift
    ;;
    -h|--help|*)
    echo "Usage example: bash get_ros2_sources.sh --distro=master --ros2-path=~/ros2_cc_ws"
    exit 0
    shift
    ;;
esac
done

echo "Using ROS2 $DISTRIBUTION distribution sources"

if [ -d $ROS2_PATH ]; then
    echo "Error: Directory $ROS2_PATH already exists, remove it to get new sources."
    exit 1
else
    # Download sources for distribution
    echo "Downloading ROS2 sources in directory: $ROS2_PATH"
    mkdir -p $ROS2_PATH/src
    cd $ROS2_PATH
    wget https://raw.githubusercontent.com/ros2/ros2/$DISTRIBUTION/ros2.repos
    vcs import src < ros2.repos
fi
