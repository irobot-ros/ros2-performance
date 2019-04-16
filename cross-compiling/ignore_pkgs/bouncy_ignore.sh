#!/bin/bash


if [ -z "$1" ]; then
    THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
    echo "Working on $THIS_DIR workspace"
else
    echo "Moving to $1 workspace"
    cd $1
fi

echo "Ignoring packages..."

sed -i -r \
    's/<build(.+?py.+?)/<\!\-\-build\1\-\->/' \
    src/ros2/rosidl_defaults/rosidl_default_generators/package.xml

touch \
    src/ros/resource_retriever/COLCON_IGNORE \
    src/ros2/demos/COLCON_IGNORE \
    src/ros2/examples/rclpy/COLCON_IGNORE \
    src/ros2/geometry2/COLCON_IGNORE \
    src/ros2/kdl_parser/COLCON_IGNORE \
    src/ros2/orocos_kinematics_dynamics/COLCON_IGNORE \
    src/ros2/rclpy/COLCON_IGNORE \
    src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE \
    src/ros2/rmw_connext/COLCON_IGNORE \
    src/ros2/rmw_opensplice/COLCON_IGNORE \
    src/ros2/robot_state_publisher/COLCON_IGNORE \
    src/ros2/ros1_bridge/COLCON_IGNORE \
    src/ros2/rosidl_python/COLCON_IGNORE \
    src/ros2/rviz/COLCON_IGNORE \
    src/ros2/system_tests/COLCON_IGNORE \
    src/ros2/urdf/COLCON_IGNORE \
    src/ros2/urdfdom/COLCON_IGNORE \
    src/ros2/rcl/rcl/test/COLCON_IGNORE \
    src/ros2/examples/rclpy/COLCON_IGNORE \
    src/ros-perception/laser_geometry/COLCON_IGNORE