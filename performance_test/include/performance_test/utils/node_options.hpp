/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__UTILS__NODE_OPTIONS_HPP_
#define PERFORMANCE_TEST__UTILS__NODE_OPTIONS_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace performance_test
{

rclcpp::NodeOptions create_node_options(
  const std::string & node_name,
  const std::string & node_namespace = "",
  const std::vector<rclcpp::Parameter> & parameters = {});

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__UTILS__NODE_OPTIONS_HPP_
