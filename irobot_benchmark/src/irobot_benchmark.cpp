/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <sys/wait.h>

#include <rclcpp/rclcpp.hpp>

#include "performance_test/communication.hpp"
#include "performance_test/performance_node.hpp"
#include "performance_test/node_types.hpp"
#include "performance_test/resource_usage_logger.hpp"
#include "performance_test/system.hpp"
#include "performance_test/utils/cli_options.hpp"
#include "performance_test/utils/fork_process.hpp"

#include "performance_test_factory/factory.hpp"

using namespace std::chrono_literals;

static void run_test(
  int argc, char** argv,
  const performance_test::Options &options,
  const std::string &topology_json,
  pid_t &pid,
  const std::string &resources_output_path,
  const std::string &events_output_path,
  const std::string &latency_all_output_path,
  const std::string &latency_total_output_path)
{
  // Start resources logger
  performance_test::ResourceUsageLogger ru_logger(resources_output_path);

  ru_logger.start(std::chrono::milliseconds(options.resources_sampling_per_ms));

  rclcpp::init(argc, argv);

  performance_test::System ros2_system(static_cast<performance_test::ExecutorType>(options.executor));

  // Get the node type from options
  auto node_type = static_cast<performance_test::NodeType>(options.node);
  // Load topology from json file
  auto factory = performance_test::TemplateFactory(options.ipc, options.ros_params, false, "", node_type);

  if (options.tracking_options.is_enabled) {
    ros2_system.enable_events_logger(events_output_path);
  }

  auto nodes_vec = factory.parse_topology_from_json(topology_json, options.tracking_options);
  ros2_system.add_node(nodes_vec);

  // now the system is complete and we can make it spin for the requested duration
  bool wait_for_discovery = true;
  ros2_system.spin(options.duration_sec, wait_for_discovery, options.name_threads);

  // terminate the experiment
  ru_logger.stop();
  rclcpp::shutdown();
  std::this_thread::sleep_for(500ms);

  ros2_system.print_latency_all_stats();

  if (options.topology_json_list.size() > 1)
  {
    std::cout << std::endl << "Process total:" << std::endl;
    ros2_system.print_latency_total_stats();
  }

  std::cout << std::endl;
  ros2_system.save_latency_all_stats(latency_all_output_path);
  ros2_system.save_latency_total_stats(latency_total_output_path);

  // Parent process: wait for children to exit and print system stats
  if (pid != 0)
  {
    if (options.topology_json_list.size() > 1)
    {
      waitpid(getpid() + 1, &pid, 0);
    }
    std::cout << "System total:" << std::endl;
    ros2_system.print_agregate_stats(options.topology_json_list);
  }
}

int main(int argc, char** argv)
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }
  int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());
  auto options = performance_test::Options(non_ros_argc, non_ros_args_c_strings.data());

  std::cout<<options<<"\n"<<"Start test!"<<std::endl;

  pid_t pid = getpid();
  size_t process_index = performance_test::fork_process(options.topology_json_list.size());

  std::string topology_json = options.topology_json_list[process_index];

  // Create results dir based on the topology name
  const size_t last_slash = topology_json.find_last_of("/");
  std::string topology_basename = topology_json.substr(last_slash + 1, topology_json.length());
  std::string dir_name = topology_basename.substr(0,topology_basename.length()-5) + "_log";

  std::string make_dir = "mkdir -p " + dir_name;
  const auto ret = system(make_dir.c_str());
  static_cast<void>(ret);
  std::string resources_output_path     = dir_name + "/resources.txt";
  std::string events_output_path        = dir_name + "/events.txt";
  std::string latency_all_output_path   = dir_name + "/latency_all.txt";
  std::string latency_total_output_path = dir_name + "/latency_total.txt";

  run_test(
    argc,
    argv,
    options,
    topology_json,
    pid,
    resources_output_path,
    events_output_path,
    latency_all_output_path,
    latency_total_output_path);
}
