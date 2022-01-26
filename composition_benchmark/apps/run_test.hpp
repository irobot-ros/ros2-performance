#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/global_factory.hpp>
#include <performance_test/utils/stat_logger.hpp>

template<typename NodeT>
using NodesVector = std::vector<std::shared_ptr<NodeT>>;

template<typename NodeT>
using create_func_t = std::function<NodesVector<NodeT> ()>;

template<typename NodeT>
using run_func_t = std::function<void (NodesVector<NodeT>)>;

template<typename NodeT>
void run_test(
  int argc,
  char** argv,
  create_func_t<NodeT> create_func = global_factory::create_nodes<NodeT>,
  run_func_t<NodeT> run_func = [](NodesVector<NodeT>){
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::hours(std::numeric_limits<int>::max()));
  })
{
  rclcpp::init(argc, argv);

  auto nodes = create_func();
  run_func(nodes);

  performance_test::log_latency_all_stats<NodeT>(std::cout, nodes);

  rclcpp::shutdown();
}
