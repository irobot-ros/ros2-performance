#include <composition_benchmark/composable_node.hpp>

ComposableNode::ComposableNode(const rclcpp::NodeOptions & options)
: performance_test::PerformanceNode<rclcpp::Node>(
  "composable_node",
  "",
  options)
{ }

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ComposableNode)
