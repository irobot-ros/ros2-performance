#include <performance_composition/base_node.hpp>
#include <performance_composition/global_factory.hpp>

BaseNode::BaseNode(const rclcpp::NodeOptions & options)
: performance_test::PerformanceNode<rclcpp::Node>(global_factory::get_node_name(), global_factory::get_namespace(), options, global_factory::get_executor_id())
{
  global_factory::mark_node_created();
}
