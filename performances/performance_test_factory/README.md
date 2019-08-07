# Performance Test Factory

This package contains a factory for creating multiple instances of `performance_test::Node` with entities of various types.

This package depends on a series of plugin, defined by the group tag `performance_test_factory_plugins`.
Each plugin will define some custom interfaces that can be used in the factory.

The simplest way of using the factory, consists in creating a json file describing the topology of a ROS2 process. Then this file can be parsed at runtime by the factory in order to create a system.

[How to create a json topology and factory plugins](create_new_topology)

### Run a system from a topology json file

```
#include "rclcpp/rclcpp.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/system.hpp"
#include "performance_test_factory/factory.hpp"

rclcpp::init(argc, argv);

performance_test::System ros2_system();
performance_test::TemplateFactory factory();

auto nodes_vec = factory.parse_topology_from_json("path_to_my_topology.json");
ros2_system.add_node(nodes_vec);

ros2_system.spin(std::chrono::seconds(10));

rclcpp::shutdown();
```

A complete example can be found in the [simple architecture example](examples/json_system_main.cpp).


### Manually create ROS2 nodes

The `performance_test::TemplateFactory` allows also to directly use strings for creating nodes, instead of parsing them from the json files.

```
#include "performance_test/ros2/node.hpp"
#include "performance_test_factory/factory.hpp"


performance_test::TemplateFactory factory();
bool use_ipc = true;
auto sub_node = factory.create_node("sub_node_name", use_ipc);
std::string msg_type = "stamped10kb";
std::string topic_name = "my_topic";
factory.add_subscriber_from_strings(sub_node, msg_type, topic_name, rmw_qos_profile_default);
```

You can also create many similar nodes all at once.
The names of all nodes, topics and services are substituted, for convenience, with numeric IDs.

The following snippet of code will create a system with 10 subscriber nodes, with names ranging from `node_0` to `node_9`, and 2 publisher nodes, with names `node_10` and `node_11` and respectively publishing on `topic_10` and `topic_11`.
Each subscriber node will subscribe to both topics.
Nodes are stored into vectors of shared pointers.
Then remember to add the nodes to the `performance_test::System` in order to run them.

```
#include "performance_test_factory/factory.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/system.hpp"

performance_test::TemplateFactory factory;

int n_publishers = 2;
int n_subscribers = 10;
float frequency = 10;
std::string msg_type = "stamped10kb";

std::vector<std::shared_ptr<performance_test::Node>> pub_nodes =
     factory.create_periodic_publisher_nodes(
          n_subscribers,
          n_subscribers + n_publishers,
          frequency,
          msg_type);

ros2_system.add_node(pub_nodes);

std::vector<std::shared_ptr<performance_test::Node>> sub_nodes =
     factory.create_subscriber_nodes(
          0,
          n_subscribers,
          n_publishers,
          msg_type);

int experiment_duration_sec = 10;
System ros2_system();

ros2_system.add_node(pub_nodes);
ros2_system.add_node(sub_nodes);
ros2_system.spin(experiment_duration_sec);
```

**NOTE:** At the moment, these factory methods for creating subscribers and clients in a node will create all the possible ones, according to the specified number of publishers/servers.
This means that using these methods, if there are 2 publishers they will publish on 2 different topics and every subscriber node will subscribe to all the available topics.
It is not possible to use the `TemplateFactory::create_subscriber_nodes` and `TemplateFactory::create_periodic_publisher_nodes` methods for creating different topologies.
If this is needed, it's better to directly use the json files.


### C++ example executables

Inside the `examples` directory you can find some C++ executables which, through command line options, allow to create different ROS2 systems.

These executables will use the `performance_test::Node` class and the `performance_test::TemplateFactory` previously described.

##### Create a process with both pubs/subs

The simplest system that you can execute is the one in which some publisher nodes and some subscriber nodes are running in a unique process.

    $ ros2 run performance_test simple_pub_sub_main --subs 5 --pubs 1 --duration 10

 - `--pubs` indicates how many nodes with a single publisher each we are going to create.
 - `--subs`  Each of them will subscribe to all the available topics.
 - `--duration` indicates how many seconds the experiment will last.

Each of these nodes will be a `performance_test::Node` object.
Once all of them have been initialized, they will start spinning and exchanging messages for the whole duration of the experiment.

##### Create separate processes for pubs/subs

Sometimes it's useful to run nodes in separate processes: for example if we want to monitor the CPU usage only of some of them or if we want to experiment different communication types.

Assume that we want to create the same system used in the last experiment, but using running all the publishers in one process and all the subscribers in another process.

We are going to use **the same command line arguments that we used in the previous test**. The only difference is that we will run two separate executables instead of one.

Let's start by creating a process with only some subscriber nodes.

    $ ros2 run performance_test subscriber_nodes_main --subs 5 --pubs 1 --duration 10

 - `--subs` indicates how many nodes with subscriptions we are going to create.
 - `--pubs` indicates how many publishers there will be. Since each publisher publishes only on one specific topic, the number of publishers and topics is equal.

With this command we are creating a process with 5 nodes and each of them will have 1 subscription
Note that using `--pubs 0` will create nodes without any subscription!

Then, it's necessary to create a new process to publish some messages.

As you have seen, we have already specified how many publishers/topics there will be in our system (with the `--pubs` argument passed to the previous command).
**NOTE**: due to this constraint, when we want to create two different processes and make them interact, we must use the same values for `--subs` and `--pubs` in both scripts.

In this case you should run

    $ ros2 run performance_test publisher_nodes_main --subs 5 --pubs 1 --duration 10

- `--subs` indicates how many subscriber nodes exist outside of this process.
- `--pubs` indicates how many nodes with one publisher each we are going to create.

With this command we are creating a process with 1 publisher node.

**NOTE**: specifying the number of publishers and subscribers in both scripts may seem redundant, but this information is used internally to let both processes be aware of how nodes and topics should be named and what elements are available outside of their process.

##### Clients and services

The same applies also for clients and services.

If you want to run them in the same process you should use the `simple_client_service_main` script, while `client_nodes_main` and `server_nodes_main` allow to create nodes of a single type.

You can control the number of nodes and of services available through the `--clients` and `--services` arguments.
They follow exactly the same logic as `--pubs` and `--subs` in the previous examples.

