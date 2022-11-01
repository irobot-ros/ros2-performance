# Performance Test Plugin CMake

This package defines the CMake function `generate_factory_plugins(MSGS SRVS)`.
This function should be invoked in a package where custom interfaces are created in order to make them usable in the performance test factory.

## How to use it

1. Create a standard ROS 2 package where new messages and services are defined.
**Important: all messages need to have a field `PerformanceHeader header` which is defined in the `performance_test_msgs` package.** 
1. Add to `package.xml`
```
<build_depend>performance_test_plugin_cmake</build_depend>
<member_of_group>performance_test_factory_plugins</member_of_group>
```
1. Add to `CMakeLists.txt`
```
find_package(performance_test_plugin_cmake REQUIRED)
generate_factory_plugin("msg/MyNewMsg.msg msg/MyOtherNewMsg.msg" "srv/MyNewSrv.srv")
```

See [irobot_interfaces_plugin](../irobot_interfaces_plugin) for an example.

## How does it work

**This tool is experimental and hacky!**

The CMake function `generate_factory_plugin` calls a Python script to automatically generate a `.cpp` file that is then used to build the C++ factory support library.
The automatically generated file can be found at `build/<PKG_NAME>/generated/<PKG_NAME>_implementation.cpp`, where `<PKG_NAME>` is the name of the package where the CMake function was called.

This example shows how some of the APIs exposed by the library should look like given for the messages defined in the [irobot_interfaces_plugin](../irobot_interfaces_plugin) package:

```
  extern "C" void add_subscriber_impl(

 std::shared_ptr<performance_test::PerformanceNodeBase> n,
    const std::string & msg_type,
    const std::string & topic_name,
    const performance_metrics::Tracker::Options & tracking_options,
    msg_pass_by_t msg_pass_by,
    const rclcpp::QoS & custom_qos_profile)
  {
    const std::map<std::string, std::function<void()>> subscribers_factory{

{ "stamped_int64", [&] { n->add_subscriber<irobot_interfaces_plugin::msg::StampedInt64>(topic_name, msg_pass_by, tracking_options, custom_qos_profile);} },
{ "stamped3_float32", [&] { n->add_subscriber<irobot_interfaces_plugin::msg::Stamped3Float32>(topic_name, msg_pass_by, tracking_options, custom_qos_profile);} },
{ "stamped4_float32", [&] { n->add_subscriber<irobot_interfaces_plugin::msg::Stamped4Float32>(topic_name, msg_pass_by, tracking_options, custom_qos_profile);} },
{ "stamped4_int32", [&] { n->add_subscriber<irobot_interfaces_plugin::msg::Stamped4Int32>(topic_name, msg_pass_by, tracking_options, custom_qos_profile);} },
{ "stamped9_float32", [&] { n->add_subscriber<irobot_interfaces_plugin::msg::Stamped9Float32>(topic_name, msg_pass_by, tracking_options, custom_qos_profile);} },
{ "stamped_vector", [&] { n->add_subscriber<irobot_interfaces_plugin::msg::StampedVector>(topic_name, msg_pass_by, tracking_options, custom_qos_profile);} }
    };

    if (subscribers_factory.find(msg_type) == subscribers_factory.end()) {
      throw std::runtime_error("unknown msg type passed to subscribers factory: " + msg_type);
    }

    subscribers_factory.at(msg_type)();
  }
```
