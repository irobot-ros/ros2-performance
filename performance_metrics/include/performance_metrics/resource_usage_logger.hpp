/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_METRICS__RESOURCE_USAGE_LOGGER_HPP_
#define PERFORMANCE_METRICS__RESOURCE_USAGE_LOGGER_HPP_

#include <malloc.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

namespace performance_metrics
{

class ResourceUsageLogger
{
public:
  struct Resources
  {
    double elasped_ms = 0;
    double cpu_usage = 0;
    uint64_t mem_arena_KB = 0;
    uint64_t mem_in_use_KB = 0;
    uint64_t mem_mmap_KB = 0;
    uint64_t mem_max_rss_KB = 0;
    uint64_t mem_virtual_KB = 0;
    uint64_t latency_us = 0;
  };

  ResourceUsageLogger() = delete;

  explicit ResourceUsageLogger(const std::string & filename, const bool csv_out = false);

  ~ResourceUsageLogger();

  void start(std::chrono::milliseconds period = std::chrono::milliseconds(1000));

  void set_get_latency_callback(std::function<uint64_t()> get_latency_fn);

  void stop();

  void print_resource_usage();

  void set_system_info(int pubs, int subs, float frequency);

private:
  // Get shared resources data
  void _get();

  void _print_header(std::ostream & stream);

  // Print data to file
  void _print(std::ostream & stream);
  template<typename T>
  void _stream_out(
    std::ostream & stream, const T val, const int space = 15, const int prec = 2,
    bool sep_suffix = true);

  Resources m_resources;
  std::fstream m_file;
  std::string m_filename;
  std::thread m_logger_thread;
  std::atomic<bool> m_is_logging;
  std::atomic<bool> m_logger_thread_done;
  std::clock_t m_t1_user;
  std::chrono::time_point<std::chrono::steady_clock> m_t1_real;
  std::chrono::time_point<std::chrono::steady_clock> m_t1_real_start;
  pid_t m_pid;
  int m_pagesize;
  std::function<uint64_t()> m_get_average_latency;

  // the following values are used for comparing different plots using the python scripts
  bool m_has_system_info {false};
  int m_pubs {0};
  int m_subs {0};
  float m_frequency {0};
  int m_wide_space {15};
  int m_narrow_space {10};
  bool m_csv_out {true};
  int m_prec {2};
};

}  // namespace performance_metrics

#endif  // PERFORMANCE_METRICS__RESOURCE_USAGE_LOGGER_HPP_
