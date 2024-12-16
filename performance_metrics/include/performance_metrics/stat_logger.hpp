/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_METRICS__STAT_LOGGER_HPP_
#define PERFORMANCE_METRICS__STAT_LOGGER_HPP_

#include <ostream>
#include <string>
#include <vector>

#include "performance_metrics/tracker.hpp"

namespace performance_metrics
{

void log_total_stats(
  uint64_t total_received,
  uint64_t total_lost,
  uint64_t total_late,
  uint64_t total_too_late,
  double average_latency,
  std::ostream & stream,
  const bool csv_out);

void log_trackers_latency_all_stats(
  std::ostream & stream,
  const std::vector<Tracker> & trackers,
  const bool csv_out = false,
  const std::string & title = "");

void log_trackers_latency_total_stats(
  std::ostream & stream,
  const std::vector<Tracker> & trackers,
  const bool csv_out = false);

uint64_t get_trackers_avg_latency(
  const std::vector<const Tracker *> & trackers);

template<typename T>
void stream_out(
  const bool csv_out,
  std::ostream & stream,
  const T val,
  const int space = 15,
  bool sep_suffix = true);

}  // namespace performance_metrics

#endif  // PERFORMANCE_METRICS__STAT_LOGGER_HPP_
