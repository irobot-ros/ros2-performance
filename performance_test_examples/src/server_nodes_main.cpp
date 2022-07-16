/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cxxopts.hpp"
#include "performance_metrics/resource_usage_logger.hpp"
#include "performance_test/system.hpp"
#include "performance_test_factory/factory.hpp"
#include "examples_options.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  auto options = ExamplesOptions(argc, argv);

  std::string events_file_path = "";
  std::string ru_file_path = "";
  std::string latency_file_path = "";
  if (!options.experiment_name.empty()) {
    latency_file_path = options.experiment_path + "/lat_rel_" + options.experiment_name + ".csv";
    if (options.monitor_stats) {
      events_file_path = options.experiment_path + "/events_" + options.experiment_name + ".csv";
      ru_file_path = options.experiment_path + "/cpu_ram_" + options.experiment_name + ".csv";
    }
  }

  performance_metrics::ResourceUsageLogger ru_logger(ru_file_path);
  ru_logger.set_system_info(options.n_services, options.n_clients, options.frequency);
  // Start resources logger
  if (options.monitor_stats) {
    ru_logger.start();
  }

  /**
   * Execution
   */

  std::cout << "Start test" << std::endl;

  rclcpp::init(argc, argv);

  performance_test_factory::TemplateFactory factory(
    options.use_ipc,
    options.use_ros_params,
    options.verbose,
    options.ros_namespace);
  performance_test::System ros2_system(
    static_cast<performance_test::ExecutorType>(options.executor),
    performance_test::SpinType::SPIN,
    events_file_path);

  auto server_nodes = factory.create_server_nodes(
    options.n_clients,
    options.n_clients + options.n_services,
    options.msg_type,
    rmw_qos_profile_default);

  ros2_system.add_nodes(server_nodes);

  std::cout << "Servers created!" << std::endl;

  // allow some time for all the nodes to be created/discovered
  std::this_thread::sleep_for(std::chrono::seconds(1));

  bool wait_discovery = false;
  ros2_system.spin(std::chrono::seconds(options.experiment_duration), wait_discovery);

  rclcpp::shutdown();

  std::this_thread::sleep_for(500ms);

  std::cout << "End test" << std::endl << std::endl;

  /**
   * Visualization
   */

  if (options.monitor_stats) {
    ru_logger.stop();
  }

  ros2_system.log_latency_all_stats();
  std::cout << std::endl;
  std::cout << "System total:" << std::endl;
  ros2_system.log_latency_total_stats();
  ros2_system.save_latency_all_stats(latency_file_path);

  return 0;
}
