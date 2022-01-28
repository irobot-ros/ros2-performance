/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <malloc.h>
#include <string>
#include <unistd.h>

namespace performance_test
{

class ResourceUsageLogger
{
public:
  struct Resources {
    double elasped_ms = 0;
    double cpu_usage = 0;
    unsigned long int mem_arena_KB = 0;
    unsigned long int mem_in_use_KB = 0;
    unsigned long int mem_mmap_KB = 0;
    unsigned long int mem_max_rss_KB = 0;
    unsigned long int mem_virtual_KB = 0;
  };

  ResourceUsageLogger() = delete;

  ResourceUsageLogger(const std::string& filename);

  void start(std::chrono::milliseconds period = std::chrono::milliseconds(1000));

  void stop();

  void print_resource_usage();

  void set_system_info(int pubs, int subs, float frequency);

private:
  // Get shared resources data
  void _get();

  void _print_header(std::ostream& stream);

  // Print data to file
  void _print(std::ostream& stream);

  Resources _resources;
  std::fstream _file;
  std::string _filename;
  std::atomic<bool> _log;
  std::atomic<bool> _done;
  std::clock_t _t1_user;
  std::chrono::time_point<std::chrono::steady_clock> _t1_real;
  std::chrono::time_point<std::chrono::steady_clock> _t1_real_start;
  pid_t _pid;
  int _pagesize;

  // the following values are used for comparing different plots using the python scripts
  bool _got_system_info = false;
  int _pubs;
  int _subs;
  float _frequency;
};

}
