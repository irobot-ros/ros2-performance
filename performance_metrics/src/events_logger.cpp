/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */
#include <sys/resource.h>
#include <sys/types.h>

#include <cmath>
#include <ctime>
#include <iomanip>
#include <string>
#include <thread>

#include "performance_metrics/events_logger.hpp"

namespace performance_metrics
{

EventsLogger::EventsLogger(const std::string & filename, const bool csv_out)
{
  m_filename = filename;
  _p_csv_out = csv_out;

  m_file.open(m_filename, std::fstream::out);
  if (!m_file.is_open()) {
    std::cout << "[EventsLogger]: Error. Could not open file " << m_filename << std::endl;
    std::cout << "[EventsLogger]: Not logging." << std::endl;
    return;
  }

  std::cout << "[EventsLogger]: Logging to " << m_filename << std::endl;
  stream_out(m_file, "Time_ms", _p_time_width);
  stream_out(m_file, "Caller", _p_caller_width);
  stream_out(m_file, "Code", _p_code_width);
  stream_out(m_file, "Description", _p_desc_width, false);

  m_file << std::endl;
}

void EventsLogger::set_start_time(std::chrono::high_resolution_clock::time_point t)
{
  m_start_time = t;
}

void EventsLogger::write_event(const Event & event)
{
  if (!m_file.is_open()) {
    return;
  }

  auto t = std::chrono::high_resolution_clock::now();

  int64_t event_timestamp_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(t - m_start_time).count();

  // the event logger can be used from multiple threads, so we add a mutex
  std::unique_lock<std::mutex> lock(m_writer_mutex);
  stream_out(m_file, event_timestamp_ms, _p_time_width);
  stream_out(m_file, event.caller_name, _p_caller_width);
  stream_out(m_file, event.code, _p_code_width);
  stream_out(m_file, event.description, _p_desc_width, false);

  m_file << std::endl;
}

template<typename T>
void EventsLogger::stream_out(
  std::ostream & stream, const T val, const int space, bool sep_suffix)
{
  // whether comma or space delimited
  if (_p_csv_out) {
    stream << val << ((sep_suffix) ? "," : "");
  } else {
    stream << std::left << std::setw(space) << std::setfill(_p_separator) << val;
  }
}

}  // namespace performance_metrics
