/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <chrono>
#include <iostream>
#include "performance_test/ros2/tracker.hpp"

#include "cxxopts.hpp"

namespace benchmark {

class Options {

public:
    Options()
    {
        ipc = true;
        duration_sec = 5;
        resources_sampling_per_ms = 500;
        tracking_options.late_percentage = 20;
        tracking_options.late_absolute_us = 5000;
        tracking_options.too_late_percentage = 100;
        tracking_options.too_late_absolute_us = 50000;
    };

    Options(int argc, char** argv)
        : Options()
    {
        parse(argc, argv);
    }

    void parse(int argc, char** argv)
    {
      // Parse arguments
      cxxopts::Options options(argv[0], "ROS2 performance benchmark");

      std::string ipc_option;

      options.add_options()
      ("h,help", "print help")
      ("ipc", "intra-process-communication",
        cxxopts::value<std::string>(ipc_option)->default_value(ipc ? "on" : "off"),"on/off")
      ("t,time", "test duration", cxxopts::value<int>(duration_sec)->default_value(std::to_string(duration_sec)),"sec")
      ("s, sampling", "resources sampling period",
        cxxopts::value<int>(resources_sampling_per_ms)->default_value(std::to_string(resources_sampling_per_ms)),"msec")
      ("late-percentage", "a msg with greater latency than this percentage of the msg publishing period is considered late",
        cxxopts::value<int>(tracking_options.late_percentage)->default_value(std::to_string(tracking_options.late_percentage)),"%")
      ("late-absolute", "a msg with greater latency than this is considered late",
        cxxopts::value<int>(tracking_options.late_absolute_us)->default_value(std::to_string(tracking_options.late_absolute_us)),"usec")
      ("too-late-percentage", "a msg with greater latency than this percentage of the msg publishing period is considered lost",
        cxxopts::value<int>(tracking_options.too_late_percentage)->default_value(std::to_string(tracking_options.too_late_percentage)),"%")
      ("too-late-absolute", "a msg with greater latency than this is considered lost",
        cxxopts::value<int>(tracking_options.too_late_absolute_us)->default_value(std::to_string(tracking_options.too_late_absolute_us)),"usec");

        try {
        auto result = options.parse(argc, argv);

        if (result.count("help")) {
          std::cout << options.help() << std::endl;
          exit(0);
        }

        if (ipc_option != "off" && ipc_option != "on") {
          throw cxxopts::argument_incorrect_type(ipc_option);
        }

      } catch (const cxxopts::OptionException& e) {
        std::cout << "Error parsing options. " << e.what() << std::endl;
        exit(1);
      }

      ipc = (ipc_option == "on" ? true : false);
    }

    bool ipc;
    int duration_sec;
    int resources_sampling_per_ms;
    performance_test::Tracker::TrackingOptions tracking_options;
};
}