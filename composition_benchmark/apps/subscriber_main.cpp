#include <composition_benchmark/base_node.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>
#include "factory_setup.hpp"
#include "run_tasks.hpp"
#include "run_test.hpp"

std::vector<std::shared_ptr<BaseNode>> create_subscriber_nodes()
{
  auto topic = performance_test::Topic<irobot_interfaces_plugin::msg::StampedVector>("my_topic");
  auto node = std::make_shared<BaseNode>();
  node->add_subscriber(topic, PASS_BY_SHARED_PTR);

  return {node};
}

int main(int argc, char** argv)
{
  generic_factory_setup(argc, argv);
  run_test<BaseNode>(argc, argv, create_subscriber_nodes, std::bind(spin_task, std::placeholders::_1, std::chrono::seconds(15)));
}
