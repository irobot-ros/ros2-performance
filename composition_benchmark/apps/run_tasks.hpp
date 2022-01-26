#pragma once

#include <future>

#include <rclcpp/rclcpp.hpp>

template<typename ExecutorT=rclcpp::executors::SingleThreadedExecutor>
void spin_task(std::vector<std::shared_ptr<BaseNode>> nodes, std::chrono::milliseconds duration)
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
void spin_future_complete_task(std::vector<std::shared_ptr<BaseNode>> nodes)
{
  auto executor = std::make_shared<ExecutorT>();

  for (auto n : nodes) {
    executor->add_node(n);
  }

  std::promise<void> promise;
  executor->spin_until_future_complete(promise.get_future());
}
