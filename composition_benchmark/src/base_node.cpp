/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include "composition_benchmark/base_node.hpp"
#include "performance_metrics/stat_logger.hpp"

BaseNode::BaseNode(
  const rclcpp::NodeOptions & options)
: performance_test::PerformanceNode<rclcpp::Node>(
  "base_node",
  "",
  options)
{ }

BaseNode::~BaseNode()
{
  performance_metrics::log_trackers_latency_all_stats(std::cout, this->sub_trackers());
}
