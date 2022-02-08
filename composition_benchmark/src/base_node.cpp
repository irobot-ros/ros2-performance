#include <composition_benchmark/base_node.hpp>
#include <performance_metrics/stat_logger.hpp>

BaseNode::BaseNode(
  const rclcpp::NodeOptions & options)
: performance_test::PerformanceNode<rclcpp::Node>(
  "base_node",
  "",
  options)
{ }

BaseNode::~BaseNode()
{
  performance_metrics::log_latency_all_stats(std::cout, this->sub_trackers());
}
