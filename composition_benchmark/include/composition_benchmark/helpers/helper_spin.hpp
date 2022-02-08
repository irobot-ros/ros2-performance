#pragma once

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <composition_benchmark/helpers/helper_types.hpp>
#include <performance_test/executors.hpp>

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_task(
  std::vector<IRobotNodePtr> nodes,
  std::chrono::milliseconds task_duration)
{
  auto executor = std::make_shared<ExecutorT>();

  for (auto n : nodes) {
    executor->add_node(n);
  }

  std::thread stop_thread([executor, task_duration]() {
    performance_test::sleep_task(task_duration);
    executor->cancel();
  });

  executor->spin();
  stop_thread.join();
}

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_isolated_task(
  std::vector<IRobotNodePtr> nodes,
  std::chrono::milliseconds task_duration)
{
  std::vector<typename ExecutorT::SharedPtr> executors;
  std::vector<std::unique_ptr<std::thread>> executor_threads;
  for (auto n : nodes) {
    auto executor = std::make_shared<ExecutorT>();
    executor->add_node(n);
    executors.push_back(executor);
    auto t = std::make_unique<std::thread>([executor]() {
      executor->spin();
    });
    executor_threads.push_back(std::move(t));
  }

  // Note: if task_duration is too small, we may end up calling cancel before
  // an executor started to sleep. This causes the function to block forever.
  performance_test::sleep_task(task_duration);

  for (auto executor : executors) {
    executor->cancel();
  }

  for (auto & thread : executor_threads) {
    thread->join();
  }
}

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_future_complete_task(
  std::vector<IRobotNodePtr> nodes,
  std::chrono::milliseconds task_duration)
{
  auto executor = std::make_shared<ExecutorT>();

  for (auto n : nodes) {
    executor->add_node(n);
  }

  std::promise<void> promise;
  std::thread stop_thread([executor, task_duration, &promise]() {
    performance_test::sleep_task(task_duration);
    promise.set_value();
    executor->cancel();
  });

  executor->spin_until_future_complete(promise.get_future());
  stop_thread.join();
}

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_some_task(
  std::vector<IRobotNodePtr> nodes,
  std::chrono::milliseconds task_duration)
{
  auto executor = std::make_shared<ExecutorT>();

  for (auto n : nodes) {
    executor->add_node(n);
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  auto duration_elapsed = [start_time, task_duration]() {
    auto now = std::chrono::high_resolution_clock::now();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return time_elapsed > task_duration;
  };

  while (rclcpp::ok() && !duration_elapsed()) {
    executor->spin_some();
  }
}
