#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <thread>

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
void spin_isolated_task(std::vector<IRobotNodePtr> nodes, std::chrono::milliseconds duration)
{
  std::vector<typename ExecutorT::SharedPtr> executors;
  std::vector<std::unique_ptr<std::thread>> executor_threads;
  for (auto n : nodes) {
    auto executor = std::make_shared<ExecutorT>();
    executor->add_node(n);
    executors.push_back(executor);
    auto t = std::make_unique<std::thread>([executor]{
      executor->spin();
    });
    executor_threads.push_back(std::move(t));
  }

  // Note: if duration is too small, we may end up calling cancel before
  // an executor started to sleep. This causes the function to block forever.
  std::this_thread::sleep_for(duration);

  for (auto executor : executors) {
    executor->cancel();
  }

  for (auto & thread : executor_threads) {
    thread->join();
  }
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
  auto duration_elapsed = [start_time, duration]() {
    auto now = std::chrono::high_resolution_clock::now();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return time_elapsed > duration;
  };

  while (rclcpp::ok() && !duration_elapsed()) {
    executor->spin_some();
  }
}
