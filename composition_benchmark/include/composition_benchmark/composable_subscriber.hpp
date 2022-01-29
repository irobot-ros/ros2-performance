#pragma once

#include <performance_test/performance_node.hpp>

class ComposableSubscriber : public performance_test::PerformanceNode<rclcpp::Node>
{
public:
  explicit ComposableSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ComposableSubscriber();
};
