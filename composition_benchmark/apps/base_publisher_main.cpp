#include <composition_benchmark/base_node.hpp>
#include <irobot_interfaces_plugin/msg/stamped_vector.hpp>
#include <composition_benchmark/helpers/helper_factory.hpp>
#include <composition_benchmark/helpers/helper_options.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>
#include <performance_test/utils/node_options.hpp>

static
std::vector<IRobotNodePtr> create_publisher_node(int argc, char ** argv)
{
  auto cli_options = CompositionOptions(argc, argv);
  auto node_options = performance_test::create_node_options(*cli_options.name);

  auto pub_period = std::chrono::milliseconds(1000 / (*cli_options.pub_frequency));

  IRobotNodePtr node = std::make_shared<BaseNode>(node_options);
  node->add_periodic_publisher<irobot_interfaces_plugin::msg::StampedVector>(
    "my_topic",
    pub_period,
    PASS_BY_UNIQUE_PTR,
    rmw_qos_profile_default,
    *cli_options.msg_size);

  return {node};
}

int main(int argc, char ** argv)
{
  run_test(
    argc,
    argv,
    create_publisher_node,
    std::bind(spin_task, std::placeholders::_1, MAX_HOURS));
}
