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

#include "performance_test/events_logger.hpp"

namespace performance_test
{

EventsLogger::EventsLogger(const std::string & filename)
{
  m_filename = filename;

  m_file.open(m_filename, std::fstream::out);
  if (!m_file.is_open()) {
    std::cout << "[EventsLogger]: Error. Could not open file " << m_filename << std::endl;
    std::cout << "[EventsLogger]: Not logging." << std::endl;
    return;
  }

  std::cout << "[EventsLogger]: Logging to " << m_filename << std::endl;

  m_file << std::left << std::setw(_p_time_width) << std::setfill(_p_separator) << "Time[ms]";
  m_file << std::left << std::setw(_p_caller_width) << std::setfill(_p_separator) << "Caller";
  m_file << std::left << std::setw(_p_code_width) << std::setfill(_p_separator) << "Code";
  m_file << std::left << std::setw(_p_desc_width) << std::setfill(_p_separator) << "Description";
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

  m_file << std::left << std::setw(_p_time_width) << std::setfill(_p_separator) <<
    event_timestamp_ms;
  m_file <<
    std::left << std::setw(_p_caller_width) << std::setfill(_p_separator) << event.caller_name;
  m_file <<
    std::left << std::setw(_p_code_width) << std::setfill(_p_separator) << event.code;
  m_file <<
    std::left << std::setw(_p_desc_width) << std::setfill(_p_separator) << event.description;
  m_file << std::endl;
}

}  // namespace performance_test
