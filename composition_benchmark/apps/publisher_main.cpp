#include <composition_benchmark/base_node.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>
#include "factory_setup.hpp"
#include "run_tasks.hpp"
#include "run_test.hpp"

std::vector<std::shared_ptr<BaseNode>> create_publisher_node()
{
  auto topic = performance_test::Topic<irobot_interfaces_plugin::msg::StampedVector>("my_topic");

  auto node = std::make_shared<BaseNode>();
  node->add_periodic_publisher<irobot_interfaces_plugin::msg::StampedVector>(
    topic,
    std::chrono::milliseconds(10),
    PASS_BY_UNIQUE_PTR,
    rmw_qos_profile_default,
    10000);

  return {node};
}

int main(int argc, char** argv)
{
  generic_factory_setup(argc, argv);
  run_test<BaseNode>(argc, argv, create_publisher_node, std::bind(spin_task, std::placeholders::_1, std::chrono::seconds(15)));
}
