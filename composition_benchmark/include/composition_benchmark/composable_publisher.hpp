#pragma once

#include <performance_test/node.hpp>

class ComposablePublisher : public performance_test::PerformanceNode<rclcpp::Node>
{
public:
  explicit ComposablePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ComposablePublisher() = default;
};
