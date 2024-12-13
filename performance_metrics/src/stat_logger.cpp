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

template<typename T>
void stream_out(
  const bool csv_out,
  std::ostream & stream,
  const T val, const int space,
  bool sep_suffix)
{
  // whether comma or space delimited
  if (csv_out) {
    stream << val << ((sep_suffix) ? "," : "");
  } else {
    const char separator = ' ';
    stream << std::left << std::setw(space) << std::setfill(separator) << val;
  }
}

void log_total_stats(
  uint64_t total_received,
  uint64_t total_lost,
  uint64_t total_late,
  uint64_t total_too_late,
  double average_latency,
  std::ostream & stream,
  const bool csv_out)
{
  const int wide_space = 15;
  const int narrow_space = 10;

  double total_lost_percentage =
    static_cast<double>(total_lost) / (total_received + total_lost) * 100;
  double total_late_percentage =
    static_cast<double>(total_late) / total_received * 100;
  double total_too_late_percentage =
    static_cast<double>(total_too_late) / total_received * 100;

  // log header
  stream_out(csv_out, stream, "received_msgs", wide_space);
  stream_out(csv_out, stream, "mean_us", narrow_space);
  stream_out(csv_out, stream, "late_msgs", narrow_space);
  stream_out(csv_out, stream, "late_perc", narrow_space);
  stream_out(csv_out, stream, "too_late_msgs", wide_space);
  stream_out(csv_out, stream, "too_late_perc", wide_space);
  stream_out(csv_out, stream, "lost_msgs", narrow_space);
  stream_out(csv_out, stream, "lost_perc", narrow_space, false);
  stream << std::endl;

  // log total values
  stream_out(csv_out, stream, total_received, wide_space);
  stream_out(csv_out, stream, average_latency, narrow_space);
  stream_out(csv_out, stream, total_late, narrow_space);
  stream_out(csv_out, stream, total_late_percentage, narrow_space);
  stream_out(csv_out, stream, total_too_late, wide_space);
  stream_out(csv_out, stream, total_too_late_percentage, wide_space);
  stream_out(csv_out, stream, total_lost, narrow_space);
  stream_out(csv_out, stream, total_lost_percentage, narrow_space, false);
  stream << std::endl;
}

void log_trackers_latency_all_stats(
  std::ostream & stream,
  const std::vector<Tracker> & trackers,
  const bool csv_out,
  const std::string & title)
{
  const char separator = ' ';
  const int wide_space = 15;
  const int narrow_space = 10;

  auto log_header = [&stream, wide_space, narrow_space, separator, csv_out](
    const std::string & header_title)
    {
      stream << std::endl;
      stream << header_title << std::endl;
      stream_out(csv_out, stream, "node", wide_space);
      stream_out(csv_out, stream, "topic", wide_space);
      stream_out(csv_out, stream, "size_b", narrow_space);
      stream_out(csv_out, stream, "received_msgs", wide_space);
      stream_out(csv_out, stream, "late_msgs", narrow_space);
      stream_out(csv_out, stream, "too_late_msgs", wide_space);
      stream_out(csv_out, stream, "lost_msgs", narrow_space);
      stream_out(csv_out, stream, "mean_us", narrow_space);
      stream_out(csv_out, stream, "sd_us", narrow_space);
      stream_out(csv_out, stream, "min_us", narrow_space);
      stream_out(csv_out, stream, "max_us", narrow_space);
      stream_out(csv_out, stream, "freq_hz", narrow_space);
      stream_out(csv_out, stream, "throughput_Kb_per_sec", wide_space, false);

      stream << std::endl;
    };

  auto log_stats_line = [&stream, wide_space, narrow_space, separator, csv_out](
    const Tracker & tracker)
    {
      stream_out(csv_out, stream, tracker.get_node_name(), wide_space);
      stream_out(csv_out, stream, tracker.get_entity_name(), wide_space);
      stream_out(csv_out, stream, tracker.size(), narrow_space);
      stream_out(csv_out, stream, tracker.received(), wide_space);
      stream_out(csv_out, stream, tracker.late(), narrow_space);
      stream_out(csv_out, stream, tracker.too_late(), wide_space);
      stream_out(csv_out, stream, tracker.lost(), narrow_space);
      stream_out(csv_out, stream, std::round(tracker.stat().mean()), narrow_space);
      stream_out(csv_out, stream, std::round(tracker.stat().stddev()), narrow_space);
      stream_out(csv_out, stream, std::round(tracker.stat().min()), narrow_space);
      stream_out(csv_out, stream, std::round(tracker.stat().max()), narrow_space);
      stream_out(csv_out, stream, tracker.frequency(), narrow_space);
      stream_out(csv_out, stream, (tracker.throughput() / 1024), wide_space, false);

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

void log_trackers_latency_total_stats(
  std::ostream & stream,
  const std::vector<Tracker> & trackers,
  const bool csv_out)
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
    average_latency, stream, csv_out);
}

uint64_t get_trackers_avg_latency(const std::vector<const Tracker *> & trackers)
{
  uint64_t total_received = 0;
  double total_latency = 0;

  for (const auto & tracker : trackers) {
    total_received += tracker->delta_received();
    total_latency += tracker->delta_received() * tracker->delta_stat().mean();
    // Reset values after read
    tracker->reset_delta_received();
    tracker->reset_delta_stat();
  }

  if (total_received) {
    return std::round(total_latency / total_received);
  } else {
    return 0;
  }
}

}  // namespace performance_metrics
