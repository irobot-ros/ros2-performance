/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef COMPOSITION_BENCHMARK__HELPERS__HELPER_FACTORY_HPP_
#define COMPOSITION_BENCHMARK__HELPERS__HELPER_FACTORY_HPP_

#include <vector>

#include "composition_benchmark/helpers/helper_types.hpp"
#include "performance_test/utils/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

template<typename NodeT>
std::vector<IRobotNodePtr>
create_simple_nodes(int argc, char ** argv)
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  std::vector<IRobotNodePtr> nodes;
  for (size_t i = 1; i < non_ros_args.size(); i ++) {
    std::string node_name = non_ros_args[i];
    auto options = performance_test::create_node_options(node_name);
    IRobotNodePtr node = std::make_shared<NodeT>(options);
    nodes.push_back(node);
  }

  return nodes;
}

#endif  // COMPOSITION_BENCHMARK__HELPERS__HELPER_FACTORY_HPP_
