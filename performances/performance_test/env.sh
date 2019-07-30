#!/bin/bash

#####
# User input data: INSERT VALID VALUES ACCORDING TO YOUR SYSTEM
#####

# Set the ROS2 SDK workspace install path:
ROS2_SDK_INSTALL_PATH="/root/ros2_ws/install"

# Set the performance test workspace install path:
ROS2_PERFORMANCE_TEST_INSTALL_PATH="/root/perf_ws/install"

# Have the workspaces (ROS2 SDK and performance test) been compiled using --merge-install option?
MERGE_INSTALL=false


#####
# env.sh script: DO NOT MODIFY BELOW THIS LINE!!!
#####

# Set package name
PERFORMANCE_TEST_PKG="performance_test"

if [ "$MERGE_INSTALL" = true ]; then
    # With --merge-install `colcon_build` creates a common directory for all libraries in the workspace
    ROS2_SDK_LIBRARIES="$ROS2_SDK_INSTALL_PATH/lib"
    ROS2_PERFORMANCE_TEST_LIBRARIES="$ROS2_PERFORMANCE_TEST_INSTALL_PATH/lib"
    ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH="$ROS2_PERFORMANCE_TEST_LIBRARIES/$PERFORMANCE_TEST_PKG"
else
    # Without --merge-install `colcon_build` creates a subdirectory for each package
    ROS2_SDK_LIBRARIES="$(find $ROS2_SDK_INSTALL_PATH -name '*.so*' -printf '%h\n' | sort -u )"
    ROS2_SDK_LIBRARIES="${ROS2_SDK_LIBRARIES//$'\n'/:}"
    ROS2_PERFORMANCE_TEST_LIBRARIES="$(find $ROS2_PERFORMANCE_TEST_INSTALL_PATH -name '*.so*' -printf '%h\n' | sort -u )"
    ROS2_PERFORMANCE_TEST_LIBRARIES="${ROS2_PERFORMANCE_TEST_LIBRARIES//$'\n'/:}"
    ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH="$ROS2_PERFORMANCE_TEST_INSTALL_PATH/$PERFORMANCE_TEST_PKG/lib/$PERFORMANCE_TEST_PKG"
fi

# Check if the specified directories exist
if [ ! -d "$ROS2_PERFORMANCE_TEST_INSTALL_PATH" ]; then
    echo "Directory $ROS2_PERFORMANCE_TEST_INSTALL_PATH doesn't exist."
    echo "Check if the ROS2_PERFORMANCE_TEST_INSTALL_PATH is correct"
    return 1
fi

if [ ! -d "$ROS2_SDK_INSTALL_PATH" ]; then
    echo "Directory $ROS2_SDK_INSTALL_PATH doesn't exist."
    echo "Check if the ROS2_SDK_INSTALL_PATH is correct"
    return 1
fi

# Add ros2 shared libraries
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS2_SDK_LIBRARIES:$ROS2_PERFORMANCE_TEST_LIBRARIES

# Specify where the C++ executables installed by this package are located
export ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH

echo "Environment sourced correctly!"