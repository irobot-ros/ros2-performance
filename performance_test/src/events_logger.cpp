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

#include <performance_test/events_logger.hpp>

namespace performance_test
{

EventsLogger::EventsLogger(const std::string & filename)
{
  _filename = filename;

  _file.open(_filename,  std::fstream::out);
  if(!_file.is_open()) {
    std::cout << "[EventsLogger]: Error. Could not open file " << _filename << std::endl;
    std::cout << "[EventsLogger]: Not logging." << std::endl;
    return;
  }

  std::cout << "[EventsLogger]: Logging to " << _filename << std::endl;

  _file << std::left << std::setw(_p_time_width) << std::setfill(_p_separator) << "Time[ms]";
  _file << std::left << std::setw(_p_caller_name_width) << std::setfill(_p_separator) << "Caller";
  _file << std::left << std::setw(_p_code_width) << std::setfill(_p_separator) << "Code";
  _file << std::left << std::setw(_p_desc_width) << std::setfill(_p_separator) << "Description" << std::endl;
}

void EventsLogger::write_event(const Event & event)
{
  if(!_file.is_open()) {
    return;
  }

  auto t = std::chrono::high_resolution_clock::now();

  int64_t event_timestamp_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t - _start_time).count();

  // the event logger can be used from multiple threads, so we add a mutex
  std::lock_guard<std::mutex> lock(_writerMutex);

  _file << std::left << std::setw(_p_time_width) << std::setfill(_p_separator) << event_timestamp_ms;
  _file << std::left << std::setw(_p_caller_name_width) << std::setfill(_p_separator) << event.caller_name;
  _file << std::left << std::setw(_p_code_width) << std::setfill(_p_separator) << event.code;
  _file << std::left << std::setw(_p_desc_width) << std::setfill(_p_separator) << event.description << std::endl;
}

}
