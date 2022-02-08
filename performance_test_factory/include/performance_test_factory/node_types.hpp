/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST_FACTORY__NODE_TYPES_HPP_
#define PERFORMANCE_TEST_FACTORY__NODE_TYPES_HPP_

#include <ostream>
#include <string>

namespace performance_test_factory
{

enum class NodeType
{
  RCLCPP_NODE = 1,
  RCLCPP_LIFECYCLE_NODE = 2,
};

inline std::ostream & operator<<(std::ostream & os, const NodeType & t)
{
  std::string node_type;
  switch (t) {
    case NodeType::RCLCPP_NODE:
      node_type = "Node";
      break;
    case NodeType::RCLCPP_LIFECYCLE_NODE:
      node_type = "LifecycleNode";
      break;
  }

  return os << node_type;
}

}  // namespace performance_test_factory

#endif  // PERFORMANCE_TEST_FACTORY__NODE_TYPES_HPP_
