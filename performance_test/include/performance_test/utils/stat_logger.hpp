#pragma once

#include <cmath>
#include <iomanip>
#include <ostream>

#include "performance_test/tracker.hpp"

namespace performance_test
{

void log_total_stats(
  unsigned long int total_received,
  unsigned long int total_lost,
  unsigned long int total_late,
  unsigned long int total_too_late,
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

template<typename NodeT>
void log_latency_all_stats(std::ostream& stream, const std::vector<std::shared_ptr<NodeT>>& nodes)
{
  const char separator = ' ';
  const int wide_space = 15;
  const int narrow_space = 10;

  auto log_header = [&stream, wide_space, narrow_space, separator]()
  {
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
    const std::string& node_name, std::pair<std::string, Tracker> tracker)
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
  stream << "Subscriptions and clients stats:"<<std::endl;
  log_header();
  for (const auto& n : nodes)
  {
    auto trackers = n->all_trackers();
    for(const auto& tracker : *trackers)
    {
      log_stats_line(n->get_name(), tracker);
    }
  }

  // Print publishers
  stream << "Publishers stats:"<<std::endl;
  log_header();
  for (const auto& n : nodes)
  {
    auto trackers = n->pub_trackers();
    for(const auto& tracker : *trackers)
    {
      log_stats_line(n->get_name(), tracker);
    }
  }
}

template<typename NodeT>
void log_latency_total_stats(std::ostream& stream, const std::vector<std::shared_ptr<NodeT>>& nodes)
{
  unsigned long int total_received = 0;
  unsigned long int total_lost = 0;
  unsigned long int total_late = 0;
  unsigned long int total_too_late = 0;
  double total_latency = 0;

  // collect total data
  for (const auto& n : nodes)
  {
    auto trackers = n->all_trackers();
    for(const auto& tracker : *trackers)
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
