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
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace performance_test
{

rclcpp::NodeOptions create_node_options(
  const std::string & node_name,
  const std::string & node_namespace = "",
  const std::vector<rclcpp::Parameter> & parameters = {});

}
