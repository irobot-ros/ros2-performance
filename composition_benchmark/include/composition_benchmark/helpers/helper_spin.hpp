#pragma once

#include <future>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/helpers/helper_types.hpp>

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_task(std::vector<IRobotNodePtr> nodes, std::chrono::milliseconds duration)
{
  auto executor = std::make_shared<ExecutorT>();

  for (auto n : nodes) {
    executor->add_node(n);
  }

  std::thread stop_thread([executor, duration]{
    std::this_thread::sleep_for(duration);
    executor->cancel();
  });

  executor->spin();
  stop_thread.join();
}

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_future_complete_task(std::vector<IRobotNodePtr> nodes, std::chrono::milliseconds duration)
{
  auto executor = std::make_shared<ExecutorT>();

  for (auto n : nodes) {
    executor->add_node(n);
  }

  std::promise<void> promise;

  std::thread stop_thread([executor, duration, &promise]{
    std::this_thread::sleep_for(duration);
    promise.set_value();
    executor->cancel();
  });

  executor->spin_until_future_complete(promise.get_future());
  stop_thread.join();
}

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_some_task(std::vector<IRobotNodePtr> nodes, std::chrono::milliseconds duration)
{
  auto executor = std::make_shared<ExecutorT>();

  for (auto n : nodes) {
    executor->add_node(n);
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  auto duration_elapsed = [start_time, duration](){
    auto now = std::chrono::high_resolution_clock::now();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return time_elapsed > duration;
  };

  while (rclcpp::ok() && !duration_elapsed()) {
    executor->spin_some();
  }
}
