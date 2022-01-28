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

#include "performance_test/utils/stat_logger.hpp"

namespace performance_test
{

void log_total_stats(
  uint64_t total_received,
  uint64_t total_lost,
  uint64_t total_late,
  uint64_t total_too_late,
  double average_latency,
  std::ostream& stream)
{
  const char separator = ' ';
  const int wide_space = 15;
  const int narrow_space = 10;

  double total_lost_percentage = (double)total_lost / (total_received + total_lost) * 100;
  double total_late_percentage = (double)total_late / total_received * 100;
  double total_too_late_percentage = (double)total_too_late / total_received * 100;

  // log header
  stream << std::left << std::setw(wide_space)   << std::setfill(separator) << "received[#]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "mean[us]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "late[#]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "late[%]";
  stream << std::left << std::setw(wide_space)   << std::setfill(separator) << "too_late[#]";
  stream << std::left << std::setw(wide_space)   << std::setfill(separator) << "too_late[%]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "lost[#]";
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "lost[%]" << std::endl;

  // log total values
  stream << std::left << std::setw(wide_space)   << std::setfill(separator) << total_received;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << average_latency;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << total_late;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << std::setprecision(4) << total_late_percentage;
  stream << std::left << std::setw(wide_space)   << std::setfill(separator) << total_too_late;
  stream << std::left << std::setw(wide_space)   << std::setfill(separator) << std::setprecision(4) << total_too_late_percentage;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << total_lost;
  stream << std::left << std::setw(narrow_space) << std::setfill(separator) << std::setprecision(4) << total_lost_percentage << std::endl;
}

void log_latency_all_stats(std::ostream& stream, const std::vector<PerformanceNodeBase*>& nodes)
{
  const char separator = ' ';
  const int wide_space = 15;
  const int narrow_space = 10;

  auto log_header = [&stream, wide_space, narrow_space, separator](const std::string & title)
  {
    stream << title <<std::endl;
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

    stream << std::endl;
  };

  auto log_stats_line = [&stream, wide_space, narrow_space, separator](
    const std::string & node_name, std::pair<std::string, Tracker> tracker)
  {
    stream << std::left << std::setw(wide_space) << std::setfill(separator) << node_name;
    stream << std::left << std::setw(wide_space) << std::setfill(separator) << tracker.first;
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << tracker.second.size();
    stream << std::left << std::setw(wide_space) << std::setfill(separator) << tracker.second.received();
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << tracker.second.late();
    stream << std::left << std::setw(wide_space) << std::setfill(separator) << tracker.second.too_late();
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << tracker.second.lost();
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << std::round(tracker.second.stat().mean());
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << std::round(tracker.second.stat().stddev());
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << std::round(tracker.second.stat().min());
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << std::round(tracker.second.stat().max());
    stream << std::left << std::setw(narrow_space) << std::setfill(separator) << tracker.second.frequency();

    stream << std::endl;
  };

  // Print all subscriptions and clients
  bool subs_header = false;
  for (const auto & n : nodes)
  {
    auto trackers = n->sub_and_client_trackers();
    for(const auto & tracker : *trackers)
    {
      if (!subs_header) {
        log_header("Subscriptions and clients stats:");
        subs_header = true;
      }
      log_stats_line(n->get_node_name(), tracker);
    }
  }

  // Print publishers
  bool pubs_header = false;
  for (const auto & n : nodes)
  {
    auto trackers = n->pub_trackers();
    for(const auto & tracker : *trackers)
    {
      if (!pubs_header) {
        log_header("Publishers stats:");
        pubs_header = true;
      }
      log_stats_line(n->get_node_name(), tracker);
    }
  }
}

void log_latency_total_stats(std::ostream& stream, const std::vector<std::shared_ptr<PerformanceNodeBase>>& nodes)
{
  uint64_t total_received = 0;
  uint64_t total_lost = 0;
  uint64_t total_late = 0;
  uint64_t total_too_late = 0;
  double total_latency = 0;

  // collect total data
  for (const auto & n : nodes)
  {
    auto trackers = n->sub_and_client_trackers();
    for(const auto & tracker : *trackers)
    {
      total_received += tracker.second.received();
      total_lost += tracker.second.lost();
      total_late += tracker.second.late();
      total_too_late += tracker.second.too_late();
      total_latency += tracker.second.received() * tracker.second.stat().mean();
    }
  }

  double average_latency = std::round(total_latency / total_received);

  log_total_stats(total_received, total_lost, total_late, total_too_late,
    average_latency, stream);
}

}
