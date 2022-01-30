#include <composition_benchmark/base_node.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>
#include <composition_benchmark/helpers/helper_factory.hpp>
#include <composition_benchmark/helpers/helper_options.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>
#include <performance_test/utils/node_options.hpp>

static
std::vector<IRobotNodePtr> create_subscriber_nodes(int argc, char ** argv)
{
  auto cli_options = CompositionOptions(argc, argv);
  auto node_options = performance_test::create_node_options(*cli_options.name);

  IRobotNodePtr node = std::make_shared<BaseNode>(node_options);
  node->add_subscriber<irobot_interfaces_plugin::msg::StampedVector>("my_topic", PASS_BY_SHARED_PTR);

  return {node};
}

int main(int argc, char ** argv)
{
  run_test(
    argc,
    argv,
    create_subscriber_nodes,
    std::bind(spin_task, std::placeholders::_1, MAX_HOURS));
}
