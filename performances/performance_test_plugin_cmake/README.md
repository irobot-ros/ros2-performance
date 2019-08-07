# Performance Test Plugin CMake

This package defines the CMake function `generate_factory_plugins(MSGS SRVS)`.
This function should be invoked in a package where custom interfaces are created in order to make them usable in the performance test factory.


The CMake function calls a Python script to generate a C++ library. This library depends on `performance_test` package.

The library is based on a `.cpp` file that can be found at `build/<PKG_NAME>/generated/<PKG_NAME>_implementation.cpp`.

This is how the file should look like.

```
#include "performance_test/ros2/node.hpp"
#include "performance_test_msgs/msg/stamped10b.hpp"
#include "performance_test_msgs/msg/stamped100b.hpp"
#include "performance_test_msgs/msg/stamped250b.hpp"
#include "performance_test_msgs/msg/stamped_vector.hpp"

void performance_test::TemplateFactory::add_subscriber_from_strings(
  std::shared_ptr<performance_test::Node> n,
  std::string msg_type,
  std::string topic_name,
  Tracker::TrackingOptions tracking_options,
  msg_pass_by_t msg_pass_by,
  rmw_qos_profile_t custom_qos_profile)
{
  const std::map<std::string, std::function<void()>>  subscribers_factory{
    {"stamped10b",         [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped10b>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
    {"stamped100b",        [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped100b>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
    {"stamped250b",        [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::Stamped250b>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } },
    {"stamped_vector",      [&] { n->add_subscriber(performance_test::Topic<performance_test_msgs::msg::StampedVector>(topic_name), msg_pass_by, tracking_options, custom_qos_profile); } }
  };

  if (subscribers_factory.find(msg_type) == subscribers_factory.end()){
    assert(0 && "unknown msg type passed to factory method!" );
  }

  subscribers_factory.at(msg_type)();
}
```