#include <composition_benchmark/composable_publisher.hpp>
#include <composition_benchmark/global_factory.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>

ComposablePublisher::ComposablePublisher(const rclcpp::NodeOptions & options)
: performance_test::PerformanceNode<rclcpp::Node>(
  global_factory::get_node_name(),
  global_factory::get_namespace(),
  options,
  global_factory::get_executor_id())
{
  auto topic_name = this->declare_parameter<std::string>("topic", "my_topic");
  auto pub_frequency = this->declare_parameter<int>("frequency", 10);
  auto msg_size = this->declare_parameter<int>("size", 10000);

  this->add_periodic_publisher<irobot_interfaces_plugin::msg::StampedVector>(
    topic_name,
    std::chrono::milliseconds(pub_frequency),
    PASS_BY_UNIQUE_PTR,
    rmw_qos_profile_default,
    msg_size);
  global_factory::mark_node_created();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ComposablePublisher)
