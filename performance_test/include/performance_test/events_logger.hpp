/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>

namespace performance_test {

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
    service_unavailable
  };

  struct Event
  {
    std::string caller_name;
    EventCode code;
    std::string description;
  };

  EventsLogger() = delete;

  EventsLogger(const std::string& filename);

  void set_start_time(std::chrono::high_resolution_clock::time_point t)
  {
    _start_time = t;
  }

  void write_event(const Event& event);

private:
  std::chrono::high_resolution_clock::time_point _start_time;
  std::fstream _file;
  std::string _filename;
  std::mutex _writerMutex;

  // For print formatting
  static const char _p_separator = ' ';
  static const int _p_time_width = 12;
  static const int _p_caller_name_width = 25;
  static const int _p_code_width = 6;
  static const int _p_desc_width = 20;
};
}