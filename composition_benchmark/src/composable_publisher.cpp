/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include "composition_benchmark/composable_publisher.hpp"
#include "irobot_interfaces_plugin/msg/stamped_vector.hpp"

ComposablePublisher::ComposablePublisher(const rclcpp::NodeOptions & options)
: performance_test::PerformanceNode<rclcpp::Node>(
  "composable_publisher",
  "",
  options)
{
  auto topic_name = this->declare_parameter<std::string>("topic", "my_topic");
  auto pub_frequency = this->declare_parameter<int>("frequency", 10);
  auto msg_size = this->declare_parameter<int>("size", 10000);

  auto period = std::chrono::milliseconds(1000 / pub_frequency);

  this->add_periodic_publisher<irobot_interfaces_plugin::msg::StampedVector>(
    topic_name,
    period,
    performance_test::msg_pass_by_t::PASS_BY_UNIQUE_PTR,
    rclcpp::SensorDataQoS(),
    msg_size);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ComposablePublisher)
