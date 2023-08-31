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

#include "performance_metrics/resource_usage_logger.hpp"

namespace performance_metrics
{

ResourceUsageLogger::ResourceUsageLogger(const std::string & filename, const bool csv_out)
: m_filename(filename), m_csv_out(csv_out)
{
  m_pid = getpid();
  m_pagesize = getpagesize();

  m_is_logging = false;
  m_logger_thread_done = true;
}

ResourceUsageLogger::~ResourceUsageLogger()
{
  this->stop();
}

void ResourceUsageLogger::start(std::chrono::milliseconds period)
{
  m_is_logging = true;

  m_file.open(m_filename, std::fstream::out);
  if (!m_file.is_open()) {
    std::cout << "[ResourceUsageLogger]: Error. Could not open file " << m_filename << std::endl;
    std::cout << "[ResourceUsageLogger]: Not logging." << std::endl;
    return;
  }

  std::cout << "[ResourceUsageLogger]: Logging to " << m_filename << std::endl;

  m_t1_real_start = std::chrono::steady_clock::now();
  m_t1_user = std::clock();
  m_t1_real = std::chrono::steady_clock::now();
  m_logger_thread_done = false;

  // create a detached thread that monitors resource usage periodically
  m_logger_thread = std::thread(
    [ = ]() {
      int64_t i = 1;
      while (m_is_logging) {
        std::this_thread::sleep_until(m_t1_real_start + period * i);
        if (i == 1) {
          _print_header(m_file);
          // print a line of zeros for better visualization
          _print(m_file);
        }
        _get();
        _print(m_file);
        i++;
      }
      m_logger_thread_done = true;
    });
}

void ResourceUsageLogger::stop()
{
  bool is_logging = m_is_logging.exchange(false);
  // Nothing to do if we were not logging
  if (!is_logging) {
    return;
  }

  if (m_logger_thread.joinable()) {
    m_logger_thread.join();
  }

  m_file.close();
}

void ResourceUsageLogger::print_resource_usage()
{
  _print_header(std::cout);
  _print(std::cout);
}

void ResourceUsageLogger::set_system_info(int pubs, int subs, float frequency)
{
  if (m_is_logging) {
    std::cout <<
      "[ResourceUsageLogger]: You have to set system info before starting the logger!" <<
      std::endl;
    return;
  }

  m_pubs = pubs;
  m_subs = subs;
  m_frequency = frequency;

  m_has_system_info = true;
}

// Get shared resources data
void ResourceUsageLogger::_get()
{
  // Get elapsed time since we started logging
  auto t2_real_start = std::chrono::steady_clock::now();
  m_resources.elasped_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(t2_real_start - m_t1_real_start).count();

  // Get CPU usage percentage
  auto t2_user = std::clock();
  auto t2_real = std::chrono::steady_clock::now();
  double time_elapsed_user_ms = 1000.0 * (t2_user - m_t1_user) / CLOCKS_PER_SEC;
  int n_threads = std::thread::hardware_concurrency();
  double time_elapsed_real_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(t2_real - m_t1_real).count();
  m_resources.cpu_usage = time_elapsed_user_ms / (time_elapsed_real_ms * n_threads) * 100;

  // Get mallinfo
#if (defined(__UCLIBC__) || defined(__GLIBC__))
  auto mem_info = mallinfo();
  m_resources.mem_arena_KB = mem_info.arena >> 10;
  m_resources.mem_in_use_KB = mem_info.uordblks >> 10;
  m_resources.mem_mmap_KB = mem_info.hblkhd >> 10;
#endif

  // Get rss
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  m_resources.mem_max_rss_KB = usage.ru_maxrss;

  // Get vsz from /proc/[pid]/statm
  std::string virtual_mem_pages_string;
  std::string statm_path = "/proc/" + std::to_string(m_pid) + "/statm";
  std::ifstream in;
  in.open(statm_path);

  if (in.is_open()) {
    in >> virtual_mem_pages_string;
    m_resources.mem_virtual_KB = ((uint64_t)std::stoi(virtual_mem_pages_string) * m_pagesize) >> 10;
    in.close();
  } else {
    m_resources.mem_virtual_KB = -1;
  }
}

template<typename T>
void ResourceUsageLogger::_stream_out(std::ostream & stream, const T val,
  const int space, const int prec, bool sep_suffix)
{
  // whether comma or space delimited
  if(m_csv_out) {
    stream << val << ((sep_suffix) ? "," : "");
  } else {
    const char separator = ' ';
    stream << std::left << std::setw(space) << std::setfill(separator) 
           << std::setprecision(prec) << val << std::defaultfloat;
  }
}

void ResourceUsageLogger::_print_header(std::ostream & stream)
{
  _stream_out(stream, "time[ms]");
  _stream_out(stream, "cpu[%]", m_narrow_space);
  _stream_out(stream, "arena[KB]");
  _stream_out(stream, "in_use[KB]");
  _stream_out(stream, "mmap[KB]");
  _stream_out(stream, "rss[KB]");
  _stream_out(stream, "vsz[KB]", m_wide_space, m_prec, m_has_system_info);

  if (m_has_system_info) {
    _stream_out(stream, "pubs");
    _stream_out(stream, "subs");
    _stream_out(stream, "frequency", false);
  }

  stream << std::endl;
}

void ResourceUsageLogger::_print(std::ostream & stream)
{
  const int prec = (m_wide_space - 1);

  _stream_out(stream, std::round(m_resources.elasped_ms), m_wide_space, prec);
  _stream_out(stream, m_resources.cpu_usage, m_narrow_space);
  _stream_out(stream, m_resources.mem_arena_KB);
  _stream_out(stream, m_resources.mem_in_use_KB);
  _stream_out(stream, m_resources.mem_mmap_KB);
  _stream_out(stream, m_resources.mem_max_rss_KB);
  _stream_out(stream, m_resources.mem_virtual_KB, m_wide_space, m_prec, m_has_system_info);

  if (m_has_system_info) {
    _stream_out(stream, m_pubs);
    _stream_out(stream, m_subs);
    _stream_out(stream, m_frequency, false);
  }

  stream << std::endl;
}

}  // namespace performance_metrics
