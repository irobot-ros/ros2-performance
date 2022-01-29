#include <composition_benchmark/base_node.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>
#include <composition_benchmark/helpers/helper_factory.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>

std::vector<IRobotNodePtr> create_publisher_node(int argc, char** argv)
{
  global_factory_generic_setup(argc, argv);

  IRobotNodePtr node = std::make_shared<BaseNode>();
  node->add_periodic_publisher<irobot_interfaces_plugin::msg::StampedVector>(
    "my_topic",
    std::chrono::milliseconds(10),
    PASS_BY_UNIQUE_PTR,
    rmw_qos_profile_default,
    10000);

  return {node};
}

int main(int argc, char** argv)
{
  run_test(argc, argv, create_publisher_node, std::bind(spin_task, std::placeholders::_1, std::chrono::seconds(15)));
}
