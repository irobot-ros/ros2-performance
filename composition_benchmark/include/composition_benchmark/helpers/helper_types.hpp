/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef COMPOSITION_BENCHMARK__HELPERS__HELPER_TYPES_HPP_
#define COMPOSITION_BENCHMARK__HELPERS__HELPER_TYPES_HPP_

#include <chrono>
#include <limits>
#include <memory>

#include "performance_test/performance_node.hpp"

using IRobotNode = performance_test::PerformanceNode<rclcpp::Node>;

using IRobotNodePtr = std::shared_ptr<IRobotNode>;

constexpr auto MAX_HOURS = std::chrono::hours(std::numeric_limits<int>::max());

#endif  // COMPOSITION_BENCHMARK__HELPERS__HELPER_TYPES_HPP_
