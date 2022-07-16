/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef COMPOSITION_BENCHMARK__HELPERS__RUN_TEST_HPP_
#define COMPOSITION_BENCHMARK__HELPERS__RUN_TEST_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "composition_benchmark/helpers/helper_types.hpp"
#include "rclcpp/rclcpp.hpp"

void run_test(
  int argc,
  char ** argv,
  create_func_t create_func,
  run_func_t run_func)
{
  rclcpp::init(argc, argv);

  auto nodes = create_func(argc, argv);
  run_func(nodes);

  rclcpp::shutdown();
}
