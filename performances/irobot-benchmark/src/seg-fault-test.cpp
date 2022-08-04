// #include <irobot_events_executor/rclcpp/executors/events_executor/events_executor.hpp>

#include "rclcpp/rclcpp.hpp"
#include "irobot_interfaces_plugin/msg/stamped10b.hpp"

using namespace std::chrono_literals;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

void topic_callback(const irobot_interfaces_plugin::msg::Stamped10b & msg)
{
  std::cout << "I heard something: " << (int)msg.data[0] << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_node");

  auto publisher = node->create_publisher<irobot_interfaces_plugin::msg::Stamped10b>("topic", 10);

  auto subscription =node->create_subscription<irobot_interfaces_plugin::msg::Stamped10b>("topic", 10, topic_callback);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);

  std::thread spinner([&](){executor->spin();});

  spinner.detach();

  irobot_interfaces_plugin::msg::Stamped10b message;
  message.data[0] = 1;

  std::cout << "Publish: " << (int)message.data[0] << std::endl;
  publisher->publish(message);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  rclcpp::shutdown();
  return 0;
}
