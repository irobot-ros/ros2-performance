/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <string>
#include <vector>

#include "performance_test/utils/node_options.hpp"

namespace performance_test
{

rclcpp::NodeOptions create_node_options(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<std::string> remap_rules = {"--ros-args"};
  if (!node_name.empty()) {
    std::string node_name_remap = std::string("__node:=") + node_name;
    remap_rules.push_back("-r");
    remap_rules.push_back(node_name_remap);
  }

  if (!node_namespace.empty()) {
    std::string node_namespace_remap = std::string("__ns:=") + node_namespace;
    remap_rules.push_back("-r");
    remap_rules.push_back(node_namespace_remap);
  }

  auto options = rclcpp::NodeOptions()
    .arguments(remap_rules)
    .parameter_overrides(parameters);

  return options;
}

}  // namespace performance_test
