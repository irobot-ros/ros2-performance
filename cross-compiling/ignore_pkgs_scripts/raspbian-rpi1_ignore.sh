#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

if [ -z "$1" ]; then
    echo "Working on $THIS_DIR workspace"
else
    echo "Moving to $1 workspace"
    cd $1
fi

echo "Ignoring packages..."

sed -i -r \
  's/<build(.+?py.+?)/<\!\-\-build\1\-\->/' \
  src/ros2/rosidl_defaults/rosidl_default_generators/package.xml

# TODO: this may print some errors if some paths are not correct, but it should still work
# we should ignore paths that do not exist
touch \
  src/eProsima/COLCON_IGNORE \
  src/micro-ROS/ros_tracing/ros2_tracing/ros2trace/COLCON_IGNORE \
  src/micro-ROS/ros_tracing/ros2_tracing/tracetools_launch/COLCON_IGNORE \
  src/micro-ROS/ros_tracing/ros2_tracing/tracetools_read/COLCON_IGNORE \
  src/micro-ROS/ros_tracing/ros2_tracing/tracetools_test/COLCON_IGNORE \
  src/micro-ROS/ros_tracing/ros2_tracing/tracetools_trace/COLCON_IGNORE \
  src/osrf/COLCON_IGNORE \
  src/ros-perception/COLCON_IGNORE \
  src/ros-planning/COLCON_IGNORE \
  src/ros/pluginlib/COLCON_IGNORE \
  src/ros/robot_state_publisher/COLCON_IGNORE \
  src/ros/ros_environment/COLCON_IGNORE \
  src/ros/ros_tutorials/COLCON_IGNORE \
  src/ros/urdfdom_headers/COLCON_IGNORE \
  src/ros2/common_interfaces/actionlib_msgs/COLCON_IGNORE \
  src/ros2/common_interfaces/diagnostic_msgs/COLCON_IGNORE \
  src/ros2/common_interfaces/shape_msgs/COLCON_IGNORE \
  src/ros2/common_interfaces/std_srvs/COLCON_IGNORE \
  src/ros2/common_interfaces/stereo_msgs/COLCON_IGNORE \
  src/ros2/common_interfaces/trajectory_msgs/COLCON_IGNORE \
  src/ros2/common_interfaces/visualization_msgs/COLCON_IGNORE \
  src/ros2/eigen3_cmake_module/COLCON_IGNORE \
  src/ros2/example_interfaces/COLCON_IGNORE \
  src/ros2/examples/rclpy/COLCON_IGNORE \
  src/ros2/examples/rclcpp/minimal_action_client/COLCON_IGNORE \
  src/ros2/examples/rclcpp/minimal_action_server/COLCON_IGNORE \
  src/ros2/examples/rclcpp/minimal_client/COLCON_IGNORE \
  src/ros2/examples/rclcpp/minimal_composition/COLCON_IGNORE \
  src/ros2/examples/rclcpp/minimal_service/COLCON_IGNORE \
  src/ros2/examples/rclcpp/minimal_timer/COLCON_IGNORE \
  src/ros2/examples/rclcpp/multithreaded_executor/COLCON_IGNORE \
  src/ros2/kdl_parser/COLCON_IGNORE \
  src/ros2/launch_ros/COLCON_IGNORE \
  src/ros2/message_filters/COLCON_IGNORE \
  src/ros2/geometry2/COLCON_IGNORE \
  src/ros2/orocos_kinematics_dynamics/COLCON_IGNORE \
  src/ros2/rclcpp/rclcpp_components/COLCON_IGNORE \
  src/ros2/rclcpp/rclcpp_lifecycle/COLCON_IGNORE \
  src/ros2/rclpy/COLCON_IGNORE \
  src/ros2/rcl_interfaces/lifecycle_msgs/COLCON_IGNORE \
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE \
  src/ros2/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE \
  src/ros2/rcl/rcl/test/COLCON_IGNORE \
  src/ros2/rcl/rcl_lifecycle/COLCON_IGNORE \
  src/ros2/realtime_support/COLCON_IGNORE \
  src/ros2/rmw_connext/COLCON_IGNORE \
  src/ros2/rmw_fastrtps/COLCON_IGNORE \
  src/ros2/rmw_opensplice/COLCON_IGNORE \
  src/ros2/rosbag2/COLCON_IGNORE \
  src/ros2/rosidl_python/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_connext/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_fastrtps/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_opensplice/COLCON_IGNORE \
  src/ros2/ros_testing/COLCON_IGNORE \
  src/ros2/ros2cli/COLCON_IGNORE \
  src/ros2/sros2/COLCON_IGNORE \
  src/ros2/system_tests/COLCON_IGNORE \
  src/ros2/tinyxml_vendor/COLCON_IGNORE \
  src/ros2/tlsf/COLCON_IGNORE \
  src/ros2/urdfdom/COLCON_IGNORE \
  src/ros2/urdf/COLCON_IGNORE \
  src/ros2/yaml_cpp_vendor/COLCON_IGNORE

patch -p0 < $THIS_DIR/fix_ros2_sources_for_rpi1.patch
