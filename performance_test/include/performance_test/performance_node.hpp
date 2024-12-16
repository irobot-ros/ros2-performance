/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__PERFORMANCE_NODE_HPP_
#define PERFORMANCE_TEST__PERFORMANCE_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "performance_test/performance_node_base.hpp"

namespace performance_test
{

template<typename NodeT = rclcpp::Node>
class PerformanceNode : public NodeT, public PerformanceNodeBase
{
public:
  PerformanceNode(
    const std::string & name,
    const std::string & ros2_namespace = "",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : NodeT(name, ros2_namespace, node_options), PerformanceNodeBase(get_node_interfaces())
  {}

  virtual ~PerformanceNode() = default;

private:
  virtual NodeInterfaces get_node_interfaces()
  {
    return NodeInterfaces {
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_parameters_interface(),
      this->get_node_services_interface(),
      this->get_node_timers_interface(),
      this->get_node_topics_interface(),
      this->get_node_waitables_interface()
    };
  }
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PERFORMANCE_NODE_HPP_
