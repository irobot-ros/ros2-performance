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
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <malloc.h>
#include <string>
#include <sys/resource.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

namespace performance_test {

class ResourceUsageLogger {

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

  ResourceUsageLogger(std::string filename) : _filename(filename)
  {
    _pid = getpid();
    _pagesize = getpagesize();

    _log = false;
    _done = true;
  };

  void start(std::chrono::milliseconds period = std::chrono::milliseconds(1000))
  {
    _file.open(_filename,  std::fstream::out);
    if(!_file.is_open()) {
      std::cout << "[ResourceUsageLogger]: Error. Could not open file " << _filename << std::endl;
      std::cout << "[ResourceUsageLogger]: Not logging." << std::endl;
      return;
    }

    std::cout << "[ResourceUsageLogger]: Logging to " << _filename << std::endl;

    _t1_real_start = std::chrono::steady_clock::now();
    _t1_user = std::clock();
    _t1_real = std::chrono::steady_clock::now();
    _log = true;
    _done = false;

    // create a detached thread that monitors resource usage periodically
    std::thread logger_thread([=]() {
      long int i = 1;
      while(this->_log) {
        std::this_thread::sleep_until(_t1_real_start + period * i);
        if (i == 1) {
          _print_header(_file);
          // print a line of zeros for better visualization
          _print(_file);
        }
        _get();
        _print(_file);
        i++;
        _done = true;
      }
    });
    logger_thread.detach();
  }

  void stop()
  {
    _log = false;
    while(_done == false); // Wait until we are done logging.
    _file.close();
  }

  void print_resource_usage()
  {
    _print_header(std::cout);
    _print(std::cout);
  }

  void set_system_info(int pubs, int subs, float frequency)
  {
    if (_log == true){
      std::cout<<"[ResourceUsageLogger]: You have to set system info before starting the logger!"<<std::endl;
      return;
    }

    _pubs = pubs;
    _subs = subs;
    _frequency = frequency;

    _got_system_info = true;
  }

private:

    // Get shared resources data
    void _get()
    {
        // Get elapsed time since we started logging
        auto t2_real_start = std::chrono::steady_clock::now();
        _resources.elasped_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2_real_start - _t1_real_start).count();

        // Get CPU usage percentage
        auto t2_user = std::clock();
        auto t2_real = std::chrono::steady_clock::now();
        double time_elapsed_user_ms = 1000.0 * (t2_user-_t1_user) / CLOCKS_PER_SEC;
        int n_threads = std::thread::hardware_concurrency();
        double time_elapsed_real_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2_real - _t1_real).count();
        _resources.cpu_usage = time_elapsed_user_ms / (time_elapsed_real_ms * n_threads) * 100;

        // Get mallinfo
#if (defined(__UCLIBC__) || defined(__GLIBC__))
        auto mem_info = mallinfo();
        _resources.mem_arena_KB = mem_info.arena >> 10;
        _resources.mem_in_use_KB = mem_info.uordblks >> 10;
        _resources.mem_mmap_KB = mem_info.hblkhd >> 10;
#endif

        // Get rss
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        _resources.mem_max_rss_KB = usage.ru_maxrss;

        // Get vsz from /proc/[pid]/statm
        std::string virtual_mem_pages_string;
        std::string statm_path = "/proc/" + std::to_string(_pid) + "/statm";
        std::ifstream in;
        in.open(statm_path);

        if(in.is_open())
        {
            in >> virtual_mem_pages_string;
            _resources.mem_virtual_KB = ((unsigned long int)std::stoi(virtual_mem_pages_string) * _pagesize) >> 10;
            in.close();
        }
        else
        {
            _resources.mem_virtual_KB = -1;
        }
    }

    void _print_header(std::ostream& stream)
    {
        const char separator = ' ';
        const int wide_space = 15;
        const int narrow_space = 10;

        stream << std::left << std::setw(wide_space) << std::setfill(separator) << "time[ms]";
        stream << std::left << std::setw(narrow_space) << std::setfill(separator) << "cpu[%]";
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << "arena[KB]";
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << "in_use[KB]";
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << "mmap[KB]";
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << "rss[KB]";
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << "vsz[KB]";

        if (_got_system_info){
            stream << std::left << std::setw(wide_space) << std::setfill(separator) << "pubs";
            stream << std::left << std::setw(wide_space) << std::setfill(separator) << "subs";
            stream << std::left << std::setw(wide_space) << std::setfill(separator) << "frequency";
        }

        stream << std::endl;
    }

    // Print data to file
    void _print(std::ostream& stream)
    {
        const char separator = ' ';
        const int wide_space = 15;
        const int narrow_space = 10;

        stream << std::left << std::setw(wide_space) << std::setfill(separator) << std::setprecision(wide_space-1) << std::round(_resources.elasped_ms);
        stream << std::left << std::setw(narrow_space) << std::setfill(separator) << std::setprecision(2) << _resources.cpu_usage;
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << _resources.mem_arena_KB;
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << _resources.mem_in_use_KB;
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << _resources.mem_mmap_KB;
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << _resources.mem_max_rss_KB;
        stream << std::left << std::setw(wide_space) << std::setfill(separator) << _resources.mem_virtual_KB;

        if (_got_system_info){
            stream << std::left << std::setw(wide_space) << std::setfill(separator) << _pubs;
            stream << std::left << std::setw(wide_space) << std::setfill(separator) << _subs;
            stream << std::left << std::setw(wide_space) << std::setfill(separator)<<  std::fixed << _frequency << std::defaultfloat;
        }

        stream << std::endl;
    }

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