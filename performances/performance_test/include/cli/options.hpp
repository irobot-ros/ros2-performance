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
#include <string>
#include "performance_test/ros2/tracker.hpp"

#include "cxxopts.hpp"

namespace benchmark {

class Options {

public:
    Options()
    {
        ipc = true;
        ros_params = false;
        duration_sec = 5;
        resources_sampling_per_ms = 500;
        tracking_options.is_enabled = false;
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
      std::string ros_params_option;
      std::string tracking_enabled_option;
      options.positional_help("FILE [FILE...]").show_positional_help();
      options.parse_positional({"topology"});
      options.add_options()
      ("h,help", "print help")
      ("topology", "json file(s) describing the topology of the system",
        cxxopts::value<std::vector<std::string>>(topology_json_list),"FILE [FILE...]")
      ("ipc", "intra-process-communication",
        cxxopts::value<std::string>(ipc_option)->default_value(ipc ? "on" : "off"),"on/off")
      ("ros_params", "enable parameter services",
        cxxopts::value<std::string>(ros_params_option)->default_value(ros_params ? "on" : "off"),"on/off")
      ("t,time", "test duration", cxxopts::value<int>(duration_sec)->default_value(std::to_string(duration_sec)),"sec")
      ("s, sampling", "resources sampling period",
        cxxopts::value<int>(resources_sampling_per_ms)->default_value(std::to_string(resources_sampling_per_ms)),"msec")
      ("tracking", "compute and logs detailed statistics and events",
        cxxopts::value<std::string>(tracking_enabled_option)->default_value(tracking_options.is_enabled ? "on" : "off"),"on/off")
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

        if(result.count("topology") == 0) {
          std::cout << "Please specify a json topology file" << std::endl;
          exit(1);
        }

        if (ipc_option != "off" && ipc_option != "on") {
          throw cxxopts::argument_incorrect_type(ipc_option);
        }

        if (tracking_enabled_option != "off" && tracking_enabled_option != "on") {
          throw cxxopts::argument_incorrect_type(tracking_enabled_option);
        }

      } catch (const cxxopts::OptionException& e) {
        std::cout << "Error parsing options. " << e.what() << std::endl;
        exit(1);
      }

      ipc = (ipc_option == "on" ? true : false);
      ros_params = (ros_params_option == "on" ? true : false);
      tracking_options.is_enabled = (tracking_enabled_option == "on" ? true : false);
    }

    bool ipc;
    bool ros_params;
    int duration_sec;
    int resources_sampling_per_ms;
    std::vector<std::string> topology_json_list;
    performance_test::Tracker::TrackingOptions tracking_options;
};
}
