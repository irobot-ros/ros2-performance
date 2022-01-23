#include <chrono>
#include <thread>

#include <performance_composition/composable_node.hpp>
#include <performance_composition/global_factory.hpp>
#include "performance_test/fork_process.hpp"
#include "performance_test/resource_usage_logger.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  global_factory::setup_nodes({
    {"node1", "", 0},
  });

  // Start resources logger
  performance_test::ResourceUsageLogger ru_logger("resources.txt");
  ru_logger.start(std::chrono::milliseconds(250));

  auto node1 = std::make_shared<ComposableNode>();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  ru_logger.stop();
  rclcpp::shutdown();

  ru_logger.print_resource_usage();
}
