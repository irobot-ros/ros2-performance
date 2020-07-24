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
    src/ros2/geometry2/examples_tf2_py/COLCON_IGNORE \
    src/ros2/geometry2/test_tf2/COLCON_IGNORE \
    src/ros2/geometry2/tf2_bullet/COLCON_IGNORE \
    src/ros2/geometry2/tf2_geometry_msgs/COLCON_IGNORE \
    src/ros2/geometry2/tf2_kdl/COLCON_IGNORE \
    src/ros2/ros1_bridge/COLCON_IGNORE \
    src/ros2/rviz/COLCON_IGNORE \
    src/ros2/demos/COLCON_IGNORE \
    src/ros2/orocos_kinematics_dynamics/COLCON_IGNORE \
    src/ros2/rosbag2/rosbag2_storage_evaluation/COLCON_IGNORE \
    src/ros/kdl_parser/COLCON_IGNORE \
    src/ros/resource_retriever/COLCON_IGNORE \
    src/ros/robot_state_publisher/COLCON_IGNORE \
    src/ros/ros_tutorials/COLCON_IGNORE \
    src/ros-visualization/COLCON_IGNORE
