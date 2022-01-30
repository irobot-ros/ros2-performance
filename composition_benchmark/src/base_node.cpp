#include <composition_benchmark/base_node.hpp>
#include <performance_test/utils/stat_logger.hpp>

BaseNode::BaseNode(
  const rclcpp::NodeOptions & options)
: performance_test::PerformanceNode<rclcpp::Node>(
  "base_node",
  "",
  options)
{ }

BaseNode::~BaseNode()
{
  auto node_ptr = dynamic_cast<performance_test::PerformanceNodeBase*>(this);
  std::vector<performance_test::PerformanceNodeBase*> nodes_vec = {node_ptr};
  performance_test::log_latency_all_stats(std::cout, nodes_vec);
}
