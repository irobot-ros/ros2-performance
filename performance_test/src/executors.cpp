/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <ostream>
#include <string>

#include "performance_test/executors.hpp"
#include "rclcpp/executors/events_executor/events_executor.hpp"

namespace performance_test
{

std::ostream & operator<<(std::ostream & os, const ExecutorType & t)
{
  std::string executor_name;
  switch (t) {
    case ExecutorType::SINGLE_THREADED_EXECUTOR:
      executor_name = "SingleThreadedExecutor";
      break;
    case ExecutorType::STATIC_SINGLE_THREADED_EXECUTOR:
      executor_name = "StaticSingleThreadedExecutor";
      break;
    case ExecutorType::EVENTS_EXECUTOR:
      executor_name = "EventsExecutor";
      break;
    default:
      executor_name = "Unknown ExecutorType";
      break;
  }

  return os << executor_name;
}

std::shared_ptr<rclcpp::Executor> make_executor(ExecutorType type)
{
  std::shared_ptr<rclcpp::Executor> executor;

  switch (type) {
    case ExecutorType::SINGLE_THREADED_EXECUTOR:
      executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      break;
    case ExecutorType::STATIC_SINGLE_THREADED_EXECUTOR:
      executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
      break;
    case ExecutorType::EVENTS_EXECUTOR:
      executor = std::make_shared<rclcpp::executors::EventsExecutor>();
      break;
  }

  return executor;
}

void sleep_task(std::chrono::milliseconds task_duration)
{
  std::mutex mtx;
  std::condition_variable cv;
  bool triggered = false;
  auto ros_context = rclcpp::contexts::get_global_default_context();
  auto callback_handle = ros_context->add_on_shutdown_callback(
    [&]() {
      {
        std::unique_lock<std::mutex> lock(mtx);
        triggered = true;
      }
      cv.notify_all();
    });

  auto wake_up_time = std::chrono::system_clock::now() + task_duration;
  std::unique_lock<std::mutex> lock(mtx);
  cv.wait_until(lock, wake_up_time, [&triggered]() {return triggered;});

  ros_context->remove_on_shutdown_callback(callback_handle);
}

}  // namespace performance_test
