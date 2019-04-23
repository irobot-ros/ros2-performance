# Developer README

Technical description of how this package works and what you should change in order to create new experiments.

## How running the experiments works

In the general [README.md](README.md) file we have described how to use some bash scripts to run batches of experiments.

What is happening under the hood is that each of these bash scripts is calling multiple times a C++ executable that creates and run a specific ROS2 system.

In the following we are going to first of all describe the most important classes and methods used in the C++ code, together with the available executables.
Then we will describe the content of the bash scripts.
Lastly we will give some examples on how to create your own bash script or how to add a new message interface to be used in your experiments.

#### Create a sample application

All the experiments use multiple instances of the `performance_test::Node` class.
This class inherits from `rclcpp::Node` and provides APIs for easily adding and storing any number of publisher, subscribers, client and servers to the node.

Each of the `Node::add_periodic_publisher`, `Node::add_subscriber`, etc methods is based on a template in order to allow the creation of systems with differnt types of messages.

We use `performance_test::Topic<MsgType>` templated structures in order to store the message type and the name of topics.
There exists an equivalent structure for services `performance_test::Service<SrvType>`

```
#include "performance_test/ros2/node.hpp"
auto topic = performance_test::Topic<performance_test_msgs::msg::Stamped10b>("my_topic_name");
auto sub_node = std::make_shared<performance_test::Node>("my_sub_node");
sub_node->add_subscriber(topic, rmw_qos_profile_default);
```

This snippet of code will create a new node called `my_sub_node` and this node will have a subscriber on `my_topic_name` where are published messages of type `10b`, i.e. a message with an header field and a statically allocated array of size 10 bytes.

Similarly you can create a second node which publishes data periodically.

```
#include "performance_test/ros2/node.hpp"
auto topic = performance_test::Topic<performance_test_msgs::msg::Stamped10b>("my_topic_name");
auto pub_node = std::make_shared<performance_test::Node>("my_pub_node");
pub_node->add_subscriber(topic, rmw_qos_profile_default);
```

**Note:** you can create nodes with any number of publishers/subscribers/clients/servers, with no constraints on their types.

Once you have defined all your nodes, you have to start them. We provide the `performance_test::System` class for managing the nodes execution.

```
#include "performance_test/ros2/system.hpp"
int experiment_duration_sec = 10;
System ros2_system(experiment_duration_sec);
ros2_system.add_node(pub_node);
ros2_system.add_node(sub_node);
ros2_system.spin();
```

This is enough for running your nodes.
While they communicate, they will internally record latency and reliability statistics.
At the end of the experiment, you can use the `performance_test::System` API to print them.

```
ros2_system.print_latency_stats();
ros2_system.save_latency_stats_to_file("my_output_file.txt");
```

Additionally, you can also monitor other type of statistics: resource usage and events.
Resource usage consists of `CPU` and several `RAM` metrics (heap, RSS, VRT).
Events on the other hand are for example the end of the discovery phase and late or lost messages.

In order to enable monitoring or resource usage you have to add the following snippet to your code:

```
#include "performance_test/ros2/resource_usage_logger.hpp"
performance_test::ResourceUsageLogger ru_logger("resource_usage_output.txt");
auto sampling_period = std::chrono::milliseconds(500);
ru_logger.start(sampling_period);

/**
 * your code to monitor goes here
 */

ru_logger.stop();
```

Lastly, in order to enable logging events, it's sufficient call the related method on the  `performance_test::System` object before starting to spin.

```
ros2_system.enable_events_logger("events_output.txt");
```

You can find a complete example in our [benchmark application package](../benchmark).

#### Use the TemplateFactory for creating flexible systems

We provide the class `performance_test::TemplateFactory` in order to allow the creation of simple, but more flexible, ROS2 systems.

In these examples the names of all nodes, topics and services are substituted, for convenience, with numeric IDs.

The simplest usage for the `performance_test::TemplateFactory` consists in dynamically creating publishers and subscribers from strings.
These strings can be passed as Command Line Arguments.

```
#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/node.hpp"
performance_test::TemplateFactory factory();
auto sub_node = std::make_shared<performance_test::Node>("my_sub_node");
std::string msg_type = "10kb";
std::string topic_name = "my_topic";
factory.add_subscriber_from_strings(sub_node, msg_type, topic_name, rmw_qos_profile_default);
```

You can publish or subscribe using any interface defined in the `performance_test_msgs` package.
Similarly there are methods for creating publishers, client and servers.

Moreover, you can use the factory in order to create multiple publishers and subscribers at the same time.

The following snippet of code will create a system with 10 subscriber nodes, with names ranging from `node_0` to `node_9`, and 2 publisher nodes, with names `node_10` and `node_11` and respectively publishing on `topic_10` and `topic_11`.
Each subscriber node will subscribe to both topics.
Nodes are stored into vectors of shared pointers.
Then remember to add the nodes to the `performance_test::System` in order to run them.

```
#include "performance_test/ros2/template_factory.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/system.hpp"

performance_test::TemplateFactory factory();

int n_publishers = 2;
int n_subscribers = 10;
float frequency = 10;
std::string msg_type = "10kb";

std::vector<std::shared_ptr<performance_test::Node>> pub_nodes =
     factory.create_periodic_publishers(
          n_subscribers,
          n_subscribers + n_publishers,
          frequency,
          msg_type);

ros2_system.add_node(pub_nodes);

std::vector<std::shared_ptr<performance_test::Node>> sub_nodes =
     factory.create_subscribers(
          0,
          n_subscribers,
          n_publishers,
          msg_type);

int experiment_duration_sec = 10;
System ros2_system(experiment_duration_sec);

ros2_system.add_node(pub_nodes);
ros2_system.add_node(sub_nodes);
ros2_system.spin();
```

**NOTE**: even if this is not currently enforced, by design `node_I` will be the only node which should publish to `topic_I` and which can provide `service_I`.
Hence each node will have at max 1 publisher and 1 server, but no limits on the number of subscribers and clients.
These are just conventions useful to make the code and the output more readable.

**NOTE:** At the moment, the factory methods for creating subscribers and clients in a node will create all the possible ones, according to the specified number of publishers/servers.
This means that using these methods, if there are 2 publishers they will publish on 2 different topics and every subscriber node will subscribe to all the available topics.
It is possible to manually create systems with a different topology by not using the `TemplateFactory::create_subscribers` and `TemplateFactory::create_periodic_publishers` methods.


#### C++ executables

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


#### Bash experiment scripts

In the `scripts` directory you can find several bash scripts which allows to execute some predefined performance tests.

Each script will usually run the aforementioned C++ executables several times, with different command line arguments.
The results will be written to one or more CSV files.

How to run these bash scripts and how to set their parameters is explained in the general [README](README.md).

The most important function used in the bash scripts is the run function. It will run a C++ executable with some particular parameters.

```
run simple_pub_sub_main --subs 5 --pubs 1 --duration 10 --filename OUTPUT_CSV_FILE
```

## Add a new interface

**NOTE:** a simple way for testing a specific message size is to just use the message type `vector` in combination of the `msg_size` value.

 - Create your new interface in the package, placing it under `performance_test_msgs/msg`.
   The constraints for the interface are that it must include a field of type `performance_test/PerformanceHeader` named `header` and an array/vector of `byte` called `data`. You can choose the size you prefer for this container.

      performance_test/interfaces/msg/blabla.msg

      performance_test_msgs/PerformanceHeader header
      byte[123] data

 - Add the new interface path to the `performance_test_msgs/CMakeLists.txt` file, inside the `CUSTOM_MSGS` list.

      /msg/blabla.msg

 - Include the generated header file in `performance_test/src/ros2/template_factory.cpp`.

      #include performance_test_msgs/msg/blabla.hpp

 - Add a map element for creating a publisher or a subscriber templated with this interface in the `TemplateFactory::add_subscriber_from_strings` and `TemplateFactory::add_periodic_publisher_from_strings` method in `performance_test/src/ros2/template_factory.cpp`.
  The key for this map element has to be a lowercase string and this is what the user will have to provide as `--msg_type` argument in order to select this new interface.

 - Add an entry with the size of this message also to the map in the `TemplateFactory::get_msg_size` method in `performance_test/src/ros2/template_factory.cpp`.

## Create a complete experiment running multiple systems

If you want to create a new experiment, you have to create a new bash script or a new C++ executable.

#### Create a new bash script

You should always begin the script with the following instructions

```
#!/bin/bash

if [ -z "$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH" ]; then
  echo 'Error: env.sh has not been sourced!' >&2
  exit 1
fi

THIS_PID=$$
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source $THIS_DIR/utility_scripts/utilities_functions.sh

OUTPUT_DIR_PATH="$THIS_DIR/../results/spin_test"
create_output_dir $OUTPUT_DIR_PATH
```

 - We ensure that the proper environment variables are set.
 - We source all the utilities function from  `utility_scripts/utilities_functions.sh`.
 - We create a folder where to store our output CSV.


If you want to run the `publisher_nodes_main` node, with `--subs 5 --pubs 1 --duration 10 --filename $OUTPUT_FILE_PATH` as command line arguments you should use the following:

```
NODE="publisher_nodes_main"
CMD="$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH/$NODE"
CMD_ARGS="--subs 5 --pubs 1 --duration 10 --filename $OUTPUT_FILE_PATH"

run $CMD $CMD_ARGS
wait
```

 - In the bash scripts we don't use the ROS2 command line interface for running nodes as it could be not available when working on ARM architectures if ROS2 has not been cross-compiled with Python3 support.
 - The `run` function will run the node in the background
 - The `wait` function will wait for all child process (i.e. the node running in the background) to end before proceeding.



## Python plot scripts

Once you have run one experiment and you got the results in a CSV, you can use the scripts contained in `scripts/plot_scripts` to display plots from the CSV.

Currently we have two different scripts for plotting process statistics and ros2 nodes statistics.

##### Plot process performance

`cpu_ram_plot.py` can be used to plot CPU and memory requirements of the process that we just launched. It requires as positional argument the path to a CSV file generated by the `run_and_monitor` bash function.

##### Plot ROS2 performance

`latency_reliability_plot.py` can be used to plot latency and reliability of ROS2 nodes.
This script requires one positional argument consisting of the path to one or more DIRECTORY containing the CSV files generated by a C++ executable.
You also must specify what you want to plot, by using the `--x` and `--y` arguments.
Available values are `subs`, `pubs`, `latency`, `reliability`, `spin_frequency`, `send_frequency`.
There are also options for plotting multiple data together.
**NOTE**: if you are plotting data which uses clients and services, you can still use these arguments. Keep in mind that `subs` refers to clients and `pubs` refers to services.


The optional argument `--y2` can be used to have two different Y axes.
For example `python ros2_performance_plot.py DIR_PATH --x subs --y latency --y2 reliability` will plot two lines, each with its own axis.
The optional argument `--separator` can be used to plot multiple data, by dividing them according to a certain value.
For example `python ros2_performance_plot.py DIR_PATH --x subs --y latency --separator send_frequency` will plot as many lines as many different `send_frequency` values we have in our CSV files.
**NOTE**: using together `--y2` and `--separator` may not always work, moreover the plot could look too cluttered.