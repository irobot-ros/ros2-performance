/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "performance_test/performance_node_base.hpp"

using namespace std::chrono_literals;

namespace performance_test
{

template<typename NodeT=rclcpp::Node>
class PerformanceNode : public NodeT, public PerformanceNodeBase
{
public:
  PerformanceNode(
    const std::string & name,
    const std::string & ros2_namespace = "",
    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(),
    int executor_id = 0)
  : NodeT(name, ros2_namespace, node_options), PerformanceNodeBase(executor_id)
  {
    this->set_ros_node(this);
    RCLCPP_INFO(this->get_logger(), "PerformanceNode %s created with executor id %d", name.c_str(), executor_id);
  }

  virtual ~PerformanceNode() = default;
};

}
