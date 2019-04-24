#pragma once

#include <atomic>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>
#include <thread>
#include <cmath>

#include <malloc.h>
#include <sys/resource.h>
#include <sys/types.h>
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
        _log = false;
        _done = true;

        _pid = getpid();
        _pagesize = getpagesize();
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
                                                if (i == 1) _print_header();
                                                _get();
                                                _print();
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


    void set_system_info(int pubs, int subs, float frequency, size_t msg_size)
    {
        if (_log){
            std::cout<<"[ResourceUsageLogger]: You have to set system info before starting the logger!"<<std::endl;
            return;
        }

        _pubs = pubs;
        _subs = subs;
        _frequency = frequency;
        _msg_size = msg_size;

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
        auto mem_info = mallinfo();
        _resources.mem_arena_KB = mem_info.arena >> 10;
        _resources.mem_in_use_KB = mem_info.uordblks >> 10;
        _resources.mem_mmap_KB = mem_info.hblkhd >> 10;

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


    void _print_header()
    {
        const char separator = ' ';
        const int cpu_width = 8;
        const int time_width = 13;
        const int mem_width = 15;

        _file << std::left << std::setw(time_width) << std::setfill(separator) << "time[ms]";
        _file << std::left << std::setw(cpu_width) << std::setfill(separator) << "cpu[%]";
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << "arena[KB]";
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << "in_use[KB]";
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << "mmap[KB]";
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << "rss[KB]";
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << "vsz[KB]";

        if (_got_system_info){
            _file << std::left << std::setw(mem_width) << std::setfill(separator) << "pubs";
            _file << std::left << std::setw(mem_width) << std::setfill(separator) << "subs";
            _file << std::left << std::setw(mem_width) << std::setfill(separator) << "frequency";
            _file << std::left << std::setw(mem_width) << std::setfill(separator) << "msg_size";
        }

        _file << std::endl;

        // print a line of zeros for better visualization
        _print();
    }


    // Print data to file
    void _print()
    {
        const char separator = ' ';
        const int cpu_width = 8;
        const int time_width = 13;
        const int mem_width = 15;

        _file << std::left << std::setw(time_width) << std::setfill(separator) << std::setprecision(time_width-1) << std::round(_resources.elasped_ms);
        _file << std::left << std::setw(cpu_width) << std::setfill(separator) << std::setprecision(2) << _resources.cpu_usage;
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << _resources.mem_arena_KB;
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << _resources.mem_in_use_KB;
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << _resources.mem_mmap_KB;
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << _resources.mem_max_rss_KB;
        _file << std::left << std::setw(mem_width) << std::setfill(separator) << _resources.mem_virtual_KB;

        if (_got_system_info){
            _file << std::left << std::setw(mem_width) << std::setfill(separator) << _pubs;
            _file << std::left << std::setw(mem_width) << std::setfill(separator) << _subs;
            _file << std::left << std::setw(mem_width) << std::setfill(separator)<<  std::fixed << _frequency << std::defaultfloat;
            _file << std::left << std::setw(mem_width) << std::setfill(separator) << _msg_size;
        }

        _file << std::endl;
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
    size_t _msg_size;
};
}