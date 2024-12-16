/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "cxxopts.hpp"
#include "performance_test/executors.hpp"
#include "performance_test_factory/cli_options.hpp"
#include "performance_test_factory/node_types.hpp"

namespace performance_test_factory
{

Options::Options()
{
  ipc = true;
  executor = 2;
  node = 1;
  ros_params = true;
  duration_sec = 5;
  csv_out = false;
  resources_sampling_per_ms = 1000;
  tracking_options.is_enabled = false;
  tracking_options.late_percentage = 20;
  tracking_options.late_absolute_us = 5000;
  tracking_options.too_late_percentage = 100;
  tracking_options.too_late_absolute_us = 50000;
}

Options::Options(int argc, char ** argv)
: Options()
{
  parse(argc, argv);
}

void Options::parse(int argc, char ** argv)
{
  // Parse arguments
  cxxopts::Options options(argv[0], "ROS2 performance benchmark");

  std::string ipc_option;
  std::string ros_params_option;
  std::string tracking_enabled_option;
  std::string csv_out_option;
  std::string result_folder_name_option;
  options.positional_help("FILE [FILE...]").show_positional_help();
  options.parse_positional({"topology"});
  options.add_options()("h,help", "print help")(
    "topology", "json file(s) describing the topology of the system",
    cxxopts::value<std::vector<std::string>>(topology_json_list), "FILE [FILE...]")(
    "ipc", "intra-process-communication",
    cxxopts::value<std::string>(ipc_option)->default_value(ipc ? "on" : "off"), "on/off")(
    "ros_params", "enable parameter services",
    cxxopts::value<std::string>(ros_params_option)->default_value(ros_params ? "on" : "off"),
    "on/off")(
    "t,time", "test duration",
    cxxopts::value<int>(duration_sec)->default_value(std::to_string(duration_sec)), "sec")(
    "s, sampling", "resources sampling period",
    cxxopts::value<int>(resources_sampling_per_ms)->default_value(
      std::to_string(
        resources_sampling_per_ms)), "msec")(
    "x, executor",
    "system executor:\n\t\t\t\t1:SingleThreadedExecutor. 2:StaticSingleThreadedExecutor. \
    3:EventsExecutor.",
    cxxopts::value<int>(executor)->default_value(std::to_string(executor)), "<1/2/3>")(
    "n, node", "the node type:\n\t\t\t\t1:Node. 2:LifecycleNode",
    cxxopts::value<int>(node)->default_value(std::to_string(node)), "<1/2>")(
    "tracking", "compute and logs detailed statistics and events",
    cxxopts::value<std::string>(tracking_enabled_option)->default_value(
      tracking_options.is_enabled ?
      "on" : "off"), "on/off")(
    "late-percentage",
    "a msg with greater latency than this percentage of the period is considered late",
    cxxopts::value<int>(tracking_options.late_percentage)->default_value(
      std::to_string(tracking_options.late_percentage)), "%")(
    "late-absolute",
    "a msg with greater latency than this is considered late",
    cxxopts::value<int>(tracking_options.late_absolute_us)->default_value(
      std::to_string(tracking_options.late_absolute_us)), "usec")(
    "too-late-percentage",
    "a msg with greater latency than this percentage of period is considered lost",
    cxxopts::value<int>(tracking_options.too_late_percentage)->default_value(
      std::to_string(tracking_options.too_late_percentage)), "%")(
    "too-late-absolute",
    "a msg with greater latency than this is considered lost",
    cxxopts::value<int>(tracking_options.too_late_absolute_us)->default_value(
      std::to_string(tracking_options.too_late_absolute_us)), "usec")(
    "csv-out",
    "write comma-delimted results files",
    cxxopts::value<std::string>(csv_out_option)->default_value(csv_out ? "on" : "off"), "on/off")(
    "results-dir", "name of the result directory (will use <topology>_log if not provided)",
    cxxopts::value<std::string>(result_folder_name_option)->default_value(""), "NAME");

  try {
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << std::endl;
      exit(0);
    }

    if (result.count("topology") == 0) {
      std::cout << "Please specify a json topology file" << std::endl;
      exit(1);
    }

    if (ipc_option != "off" && ipc_option != "on") {
      throw cxxopts::argument_incorrect_type(ipc_option);
    }

    if (tracking_enabled_option != "off" && tracking_enabled_option != "on") {
      throw cxxopts::argument_incorrect_type(tracking_enabled_option);
    }

    if (csv_out_option != "off" && csv_out_option != "on") {
      throw cxxopts::argument_incorrect_type(csv_out_option);
    }
    if (result_folder_name_option != "" && (result.count("topology") == 1)) {
      // Only allow to set folder name if a single topology passed
      result_folder_name = result_folder_name_option;
    }
  } catch (const cxxopts::OptionException & e) {
    std::cout << "Error parsing options. " << e.what() << std::endl;
    exit(1);
  }

  ipc = (ipc_option == "on" ? true : false);
  ros_params = (ros_params_option == "on" ? true : false);
  tracking_options.is_enabled = (tracking_enabled_option == "on" ? true : false);
  csv_out = (csv_out_option == "on" ? true : false);
}

std::ostream & operator<<(std::ostream & os, const Options & options)
{
  os << "topology_json_list:";
  for (const auto & json : options.topology_json_list) {
    os << " " << json;
  }
  os << std::endl;

  // Get the system executor from options
  auto system_executor = static_cast<performance_test::ExecutorType>(options.executor);
  os << "system_executor: " << system_executor << std::endl;

  // Get the node type from options
  auto node_type = static_cast<performance_test_factory::NodeType>(options.node);
  os << "node_type: " << node_type << std::endl;
  os << "ipc: " << (options.ipc ? "on" : "off") << std::endl;
  os << "ros_params: " << (options.ros_params ? "on" : "off") << std::endl;
  os << "duration_sec: " << options.duration_sec << " seconds" << std::endl;
  os << "resources_sampling_per_ms: " << options.resources_sampling_per_ms << std::endl;
  os << "csv_out: " << (options.csv_out ? "on" : "off") << std::endl;
  os << "tracking.is_enabled: " << (options.tracking_options.is_enabled ? "on" : "off")
     << std::endl;
  if (options.result_folder_name != "" && options.topology_json_list.size() == 1) {
    os << "results-dir: " << options.result_folder_name << std::endl;
  }
  return os;
}

}  // namespace performance_test_factory
