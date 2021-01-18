#!/bin/bash

# You can specify a distribution with --distro=<distribution>, otherwise the default one will be used
DISTRIBUTION="foxy"

# You can specify a directory to store sources with --ros2-path=<DIR>, otherwise the default one will be used
ROS2_PATH=~/ros2_cc_ws

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

for i in "$@"
do
case $i in
    --distro=*)
    DISTRIBUTION="${i#*=}"
    shift
    ;;
    --ros2-path=*)
    ROS2_PATH="${i#*=}"
    # correctly expand ~ token
    ROS2_PATH="${ROS2_PATH/#\~/$HOME}"
    shift
    ;;
    -h|--help|*)
    echo "Usage example: bash get_ros2_sources.sh --distro=master --ros2-path=~/ros2_cc_ws"
    exit 0
    shift
    ;;
esac
done

if [ -z "$TARGET_ARCHITECTURE" ]; then
    echo "Missing TARGET_ARCHITECTURE environment variables. Please run first"
    echo "source env.sh <TARGET_ARCHITECTURE>";
    exit 1;
fi

if [ -d $ROS2_PATH ]; then
    echo "Error: Directory $ROS2_PATH already exists, remove it before calling the script."
    exit 1
fi
# Create the workspace directory
mkdir -p $ROS2_PATH/src

echo "Getting ROS 2 sources for distribution: $DISTRIBUTION"
echo "Sources will be downloaded at: $ROS2_PATH"

ARCHITECTURE_SPECIFIC_SCRIPT=$THIS_DIR/get_sources/$TARGET_ARCHITECTURE"_get_ros2_sources.sh"
echo $ARCHITECTURE_SPECIFIC_SCRIPT
if [ -f $ARCHITECTURE_SPECIFIC_SCRIPT ]; then
    echo "Using configuration for architecture: $TARGET_ARCHITECTURE"
    bash $ARCHITECTURE_SPECIFIC_SCRIPT $DISTRIBUTION $ROS2_PATH
else
    # Default logic for getting ROS 2 sources
    echo "Downloading default sources for architecture: $TARGET_ARCHITECTURE"
    cd $ROS2_PATH
    wget https://raw.githubusercontent.com/ros2/ros2/$DISTRIBUTION/ros2.repos
    vcs import src < ros2.repos
fi
