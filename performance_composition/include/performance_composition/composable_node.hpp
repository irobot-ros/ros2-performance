#pragma once

#include <performance_test/node.hpp>

class ComposableNode : public performance_test::PerformanceNode<rclcpp::Node>
{
public:
    explicit ComposableNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~ComposableNode() = default;
};
