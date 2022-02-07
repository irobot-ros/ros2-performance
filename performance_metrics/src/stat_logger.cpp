/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <cmath>
#include <iomanip>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "performance_metrics/stat_logger.hpp"

namespace performance_metrics
{

void log_total_stats(
  uint64_t total_received,
  uint64_t total_lost,
  uint64_t total_late,
  uint64_t total_too_late,
  double average_latency,
  std::ostream & stream)
{
  const char separator = ' ';
  const int wide_space = 15;
  const int narrow_space = 10;

  double total_lost_percentage =
    static_cast<double>(total_lost) / (total_received + total_lost) * 100;
  double total_late_percentage =
    static_cast<double>(total_late) / total_received * 100;
  double total_too_late_percentage =
    static_cast<double>(total_too_late) / total_received * 100;

  // log header
  stream << std::left << std::setw(wide_space) << std::setfill(separator) << "received[#]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "mean[us]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "late[#]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "late[%]";
  stream << std::left << std::setw(wide_space) << std::setfill(separator) << "too_late[#]";
  stream << std::left << std::setw(wide_space) << std::setfill(separator) << "too_late[%]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "lost[#]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "lost[%]";
  stream << std::endl;

  // log total values
  stream << std::left << std::setw(wide_space) << std::setfill(separator) << total_received;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << average_latency;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << total_late;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
    std::setprecision(4) << total_late_percentage;
  stream << std::left << std::setw(wide_space) << std::setfill(separator) << total_too_late;
  stream << std::left << std::setw(wide_space) << std::setfill(separator) <<
    std::setprecision(4) << total_too_late_percentage;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << total_lost;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
    std::setprecision(4) << total_lost_percentage;
  stream << std::endl;
}

void log_latency_all_stats(
  std::ostream & stream,
  const std::vector<Tracker> & trackers,
  const std::string & title)
{
  const char separator = ' ';
  const int wide_space = 15;
  const int narrow_space = 10;

  auto log_header = [&stream, wide_space, narrow_space, separator](const std::string & header_title)
    {
      stream << header_title << std::endl;
      stream << std::left << std::setw(wide_space) << std::setfill(separator) << "node";
      stream << std::left << std::setw(wide_space) << std::setfill(separator) << "topic";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "size[b]";
      stream << std::left << std::setw(wide_space) << std::setfill(separator) << "received[#]";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "late[#]";
      stream << std::left << std::setw(wide_space) << std::setfill(separator) << "too_late[#]";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "lost[#]";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "mean[us]";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "sd[us]";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "min[us]";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "max[us]";
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "freq[hz]";
      stream << std::left << std::setw(wide_space) << std::setfill(separator) <<
        "throughput[Mb/s]";

      stream << std::endl;
    };

  auto log_stats_line = [&stream, wide_space, narrow_space, separator](
    const Tracker & tracker)
    {
      stream << std::left << std::setw(wide_space) << std::setfill(separator) <<
        tracker.get_node_name();
      stream << std::left << std::setw(wide_space) << std::setfill(separator) <<
        tracker.get_entity_name();
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        tracker.size();
      stream << std::left << std::setw(wide_space) << std::setfill(separator) <<
        tracker.received();
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        tracker.late();
      stream << std::left << std::setw(wide_space) << std::setfill(separator) <<
        tracker.too_late();
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        tracker.lost();
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        std::round(tracker.stat().mean());
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        std::round(tracker.stat().stddev());
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        std::round(tracker.stat().min());
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        std::round(tracker.stat().max());
      stream << std::left << std::setw(narrow_space) << std::setfill(separator) <<
        tracker.frequency();
      stream << std::left << std::setw(wide_space) << std::setfill(separator) <<
        (tracker.throughput() / (1024 * 1024));

      stream << std::endl;
    };

  if (trackers.empty()) {
    return;
  }

  log_header(title);
  for (const auto & tracker : trackers) {
    log_stats_line(tracker);
  }
}

void log_latency_total_stats(
  std::ostream & stream,
  const std::vector<Tracker> & trackers)
{
  uint64_t total_received = 0;
  uint64_t total_lost = 0;
  uint64_t total_late = 0;
  uint64_t total_too_late = 0;
  double total_latency = 0;

  // collect total data
  for (const auto & tracker : trackers) {
    total_received += tracker.received();
    total_lost += tracker.lost();
    total_late += tracker.late();
    total_too_late += tracker.too_late();
    total_latency += tracker.received() * tracker.stat().mean();
  }

  double average_latency = std::round(total_latency / total_received);

  log_total_stats(
    total_received, total_lost, total_late, total_too_late,
    average_latency, stream);
}

}  // namespace performance_metrics
