#pragma once

#include <ostream>
#include <vector>

#include "performance_test/node.hpp"
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
  for (const auto& n : nodes) {
    nodes_raw.push_back(n.get());
  }

  log_latency_all_stats(stream, nodes_raw);
}

void log_latency_total_stats(std::ostream& stream, const std::vector<std::shared_ptr<PerformanceNodeBase>>& nodes);

}
