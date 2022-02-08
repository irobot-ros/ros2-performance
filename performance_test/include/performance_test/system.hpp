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
#include <string>
#include <vector>

#include "performance_metrics/events_logger.hpp"
#include "performance_test/performance_node_base.hpp"
#include "performance_test/executors.hpp"

namespace performance_test
{

class System
{
public:
  explicit System(ExecutorType executor);

  template<typename NodeT>
  void add_node(const std::vector<std::shared_ptr<NodeT>> & nodes)
  {
    for (auto node : nodes) {
      this->add_node(node);
    }
  }

  void add_node(std::shared_ptr<performance_test::PerformanceNodeBase> node);

  void spin(int duration_sec, bool wait_for_discovery = true, bool name_threads = true);

  void enable_events_logger(const std::string & events_logger_path);

  void save_latency_all_stats(const std::string & filename) const;

  void save_latency_total_stats(const std::string & filename) const;

  void log_latency_all_stats(std::ostream & stream = std::cout) const;

  void log_latency_total_stats(std::ostream & stream = std::cout) const;

  void print_agregate_stats(const std::vector<std::string> & topology_json_list) const;

private:
  void wait_discovery();

  void wait_pdp_discovery(
    std::chrono::milliseconds period = std::chrono::milliseconds(20),
    std::chrono::milliseconds max_pdp_time = std::chrono::milliseconds(30 * 1000));

  void wait_edp_discovery(
    std::chrono::milliseconds period = std::chrono::milliseconds(20),
    std::chrono::milliseconds max_edp_time = std::chrono::milliseconds(30 * 1000));

  std::chrono::high_resolution_clock::time_point m_start_time;

  int m_experiment_duration_sec;

  std::vector<std::shared_ptr<performance_test::PerformanceNodeBase>> m_nodes;

  std::map<int, NamedExecutor> m_executors_map;

  std::shared_ptr<performance_metrics::EventsLogger> m_events_logger;

  ExecutorType m_executor_type;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__SYSTEM_HPP_
