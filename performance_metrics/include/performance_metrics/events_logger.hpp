/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_METRICS__EVENTS_LOGGER_HPP_
#define PERFORMANCE_METRICS__EVENTS_LOGGER_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>

namespace performance_metrics
{

/**
 * This class implements a synchronous writer for events.
 * It's thread safe and logs each event on a different line of a TSV file.
 */
class EventsLogger
{
public:
  enum EventCode
  {
    discovery,
    late_message,
    too_late_message,
    lost_messages,
    service_unavailable,
    action_unavailable
  };

  struct Event
  {
    std::string caller_name;
    EventCode code;
    std::string description;
  };

  EventsLogger() = delete;

  explicit EventsLogger(const std::string & filename, const bool csv_out = false);

  void set_start_time(std::chrono::high_resolution_clock::time_point t);

  void write_event(const Event & event);

private:
  template<typename T>
  void stream_out(
    std::ostream & stream, const T val, const int space = 15, bool sep_suffix = true);

  std::chrono::high_resolution_clock::time_point m_start_time;
  std::fstream m_file;
  std::string m_filename;
  std::mutex m_writer_mutex;

  // For print formatting
  static const char _p_separator = ' ';
  static const int _p_time_width = 12;
  static const int _p_caller_width = 25;
  static const int _p_code_width = 6;
  static const int _p_desc_width = 20;
  bool _p_csv_out = false;
};

}  // namespace performance_metrics

#endif  // PERFORMANCE_METRICS__EVENTS_LOGGER_HPP_
