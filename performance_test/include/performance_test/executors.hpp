/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#ifndef PERFORMANCE_TEST__EXECUTORS_HPP_
#define PERFORMANCE_TEST__EXECUTORS_HPP_

#include <chrono>
#include <memory>
#include <ostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace performance_test
{

struct NamedExecutor
{
  std::shared_ptr<rclcpp::Executor> executor;
  std::string name;
};

enum class ExecutorType
{
  SINGLE_THREADED_EXECUTOR = 1,
  STATIC_SINGLE_THREADED_EXECUTOR = 2,
  EVENTS_EXECUTOR = 3,
  EVENTS_EXECUTOR_EXPERIMENTAL_NS = 4,
};

enum class SpinType
{
  SPIN = 1,
  SPIN_SOME = 2,
  SPIN_FUTURE_COMPLETE = 3,
};

std::ostream & operator<<(std::ostream & os, const ExecutorType & t);

std::shared_ptr<rclcpp::Executor> make_executor(ExecutorType type);

void sleep_task(std::chrono::milliseconds task_duration);

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__EXECUTORS_HPP_
