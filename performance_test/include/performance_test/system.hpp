/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__SYSTEM_HPP_
#define PERFORMANCE_TEST__SYSTEM_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "performance_metrics/events_logger.hpp"
#include "performance_metrics/resource_usage_logger.hpp"
#include "performance_test/performance_node_base.hpp"
#include "performance_test/executors.hpp"

namespace performance_test
{

class System
{
public:
  explicit System(
    ExecutorType executor_type = ExecutorType::SINGLE_THREADED_EXECUTOR,
    SpinType spin_type = SpinType::SPIN,
    const std::optional<std::string> & events_logger_path = std::nullopt,
    const bool csv_out = false);

  ~System();

  template<typename NodeT>
  void add_nodes(const std::vector<std::shared_ptr<NodeT>> & nodes)
  {
    for (auto & node : nodes) {
      this->add_node(node);
    }
  }

  void add_node(std::shared_ptr<performance_test::PerformanceNodeBase> node);

  void spin(
    std::chrono::seconds duration,
    bool wait_for_discovery = true);

  void save_latency_all_stats(
    const std::string & filename,
    bool include_services = true) const;

  void save_latency_total_stats(
    const std::string & filename,
    bool include_services = true) const;

  void log_latency_all_stats(
    std::ostream & stream = std::cout,
    bool include_services = true) const;

  void log_latency_total_stats(
    std::ostream & stream = std::cout,
    bool include_services = true) const;

  void print_aggregate_stats(
    const std::vector<std::string> & topology_json_list,
    const std::string & results_folder_path) const;

  void set_latency_callback(
    performance_metrics::ResourceUsageLogger & ru_logger);

private:
  void wait_discovery();

  void wait_pdp_discovery(
    std::chrono::milliseconds period = std::chrono::milliseconds(20),
    std::chrono::milliseconds max_pdp_time = std::chrono::milliseconds(30 * 1000));

  void wait_edp_discovery(
    std::chrono::milliseconds period = std::chrono::milliseconds(20),
    std::chrono::milliseconds max_edp_time = std::chrono::milliseconds(30 * 1000));

  std::unique_ptr<std::thread> create_spin_thread(rclcpp::Executor::SharedPtr executor);

  std::chrono::high_resolution_clock::time_point m_start_time;

  std::chrono::seconds m_experiment_duration;

  std::vector<const performance_metrics::Tracker *> m_all_subscription_trackers;
  std::vector<std::shared_ptr<performance_test::PerformanceNodeBase>> m_nodes;
  std::map<int, NamedExecutor> m_executors_map;
  std::vector<std::unique_ptr<std::thread>> m_threads;
  std::promise<void> m_promise;

  std::shared_ptr<performance_metrics::EventsLogger> m_events_logger;

  ExecutorType m_executor_type;
  SpinType m_spin_type;
  bool m_csv_out;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__SYSTEM_HPP_
