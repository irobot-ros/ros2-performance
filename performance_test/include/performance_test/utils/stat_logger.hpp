/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <ostream>
#include <vector>

#include "performance_test/performance_node_base.hpp"
#include "performance_test/tracker.hpp"

namespace performance_test
{

void log_total_stats(
  unsigned long int total_received,
  unsigned long int total_lost,
  unsigned long int total_late,
  unsigned long int total_too_late,
  double average_latency,
  std::ostream& stream);

void log_latency_all_stats(std::ostream& stream, const std::vector<PerformanceNodeBase*>& nodes);

template<typename NodeT>
void log_latency_all_stats(std::ostream& stream, const std::vector<std::shared_ptr<NodeT>>& nodes)
{
  std::vector<NodeT*> nodes_raw;
  for (const auto & n : nodes) {
    nodes_raw.push_back(n.get());
  }

  log_latency_all_stats(stream, nodes_raw);
}

void log_latency_total_stats(std::ostream& stream, const std::vector<std::shared_ptr<PerformanceNodeBase>>& nodes);

}
