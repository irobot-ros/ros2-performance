#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/helpers/helper_types.hpp>

using NodesVector = std::vector<IRobotNodePtr>;

using create_func_t = std::function<NodesVector (int, char**)>;

using run_func_t = std::function<void (const NodesVector&)>;

void run_test(
  int argc,
  char** argv,
  create_func_t create_func,
  run_func_t run_func = [](const NodesVector&) {
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::hours(std::numeric_limits<int>::max()));
  })
{
  rclcpp::init(argc, argv);

  auto nodes = create_func(argc, argv);
  run_func(nodes);

  rclcpp::shutdown();
}
