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

class EventsLogger {

public:

    enum EventCode {
        discovery,
        late_message,
        too_late_message,
        lost_messages
    };

    struct Event {
        std::string caller_name;
        EventCode code;
        std::string description;
    };


    EventsLogger() = delete;

    EventsLogger(std::string filename)
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
    };


    void set_start_time(std::chrono::high_resolution_clock::time_point t)
    {
        _start_time = t;
    }


    void write_event(Event event)
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