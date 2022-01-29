#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/helpers/helper_types.hpp>

void run_test(
  int argc,
  char** argv,
  create_func_t create_func,
  run_func_t run_func)
{
  rclcpp::init(argc, argv);

  auto nodes = create_func(argc, argv);
  run_func(nodes);

  rclcpp::shutdown();
}
