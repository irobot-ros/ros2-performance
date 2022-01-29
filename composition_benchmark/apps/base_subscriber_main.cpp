#include <composition_benchmark/base_node.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>
#include <composition_benchmark/helpers/helper_factory.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>

std::vector<IRobotNodePtr> create_subscriber_nodes(int argc, char** argv)
{
  global_factory_generic_setup(argc, argv);
  IRobotNodePtr node = std::make_shared<BaseNode>();
  node->add_subscriber<irobot_interfaces_plugin::msg::StampedVector>("my_topic", PASS_BY_SHARED_PTR);

  return {node};
}

int main(int argc, char** argv)
{
  run_test(
    argc,
    argv,
    create_subscriber_nodes,
    std::bind(spin_task, std::placeholders::_1, MAX_HOURS));
}
