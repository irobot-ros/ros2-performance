#pragma once

#include <performance_test/node.hpp>

class BaseNode : public performance_test::PerformanceNode<rclcpp::Node>
{
public:
  explicit BaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~BaseNode() = default;
};
