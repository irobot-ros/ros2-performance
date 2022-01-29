#pragma once

#include <chrono>
#include <functional>
#include <limits>
#include <vector>

#include <performance_test/performance_node.hpp>

using IRobotNode = performance_test::PerformanceNode<rclcpp::Node>;

using IRobotNodePtr = std::shared_ptr<IRobotNode>;

using NodesVector = std::vector<IRobotNodePtr>;

using create_func_t = std::function<NodesVector (int, char**)>;

using run_func_t = std::function<void (const NodesVector&)>;

constexpr auto MAX_HOURS = std::chrono::hours(std::numeric_limits<int>::max());
