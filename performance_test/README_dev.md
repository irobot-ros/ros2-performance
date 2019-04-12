# Developer README

Technical description of how this package works and what you should change in order to create new experiments.

## How running the experiments works

In the general [README.md](README.md) file we have described how to use some bash scripts to run batches of experiments.

What is happening under the hood is that each of these bash scripts is calling several times a C++ executable that creates and run a specific ROS2 system.

In the following we are going to first of all describe the most important classes and methods used in the C++ code, together with the available executables.
Then we will describe the content of the bash scripts.
Lastly we will give some examples on how to create your own bash script or how to add a new message interface to be used in your experiments.

#### TemplatedMultiNode class

All the experiments use multiple instances of the `TemplatedMultiNode` class.
This class inherits from `rclcpp::Node` and provides APIs for easily adding any number of publisher, subscribers, client and servers to the node.
It's based on a template which requires to specify which type of message should be used by the publishers and subscribers of the node.

**NOTE:** This means that at the moment all the publishers and subscribers on a given node must be of the same message type. If you want to have messages of different size you can use a dynamically allocated message type (i.e. `vector`) and specify a different message size for each publisher, as you will see in the following instructions.

**NOTE:** At the moment the service implementation is not templated, i.e. only services of type `GetHeader` can be created.


All the names in the ROS graph (node names, topics names, service names) can be substituted, for convenience, with numeric IDs.
Nodes can be created using some helper functions.

```
#include "performance_test/ros2/template_factory.hpp"
int node_id = 1;
int topic_id = 1;
std::string msg_type = "10b"
TemplateFactory factory();
auto node = factory.make_templated(msg_type, node_id);
node->add_publisher(topic_id);
```

This snippet of code will create a new node called `node_1` and this node will have a publisher on `topic_1` which publishes messages of type `10b`, i.e. a message with an header field and a statically allocated array of size 10 bytes.

Similarly you can create a second node, with a different name, which subscribes to this topic

```
#include "performance_test/ros2/template_factory.hpp"
int node_id = 2;
int topic_id = 1;
std::string msg_type = "10b"
TemplateFactory factory();
auto node = factory.make_templated(msg_type, node_id);
node->add_subscriber(topic_id);
```

**NOTE**: even if this is not currently enforced, by design `node_I` will be the only node which should publish to `topic_I` and which can provide `service_I`.
Hence each node will have at max 1 publisher and 1 server, but no limits on the number of subscribers and clients.
These are just conventions useful to make the code and the output more readable.


Each `TemplatedMultiNode` object will collect statistics while it's running and it will store them into an internal data structure of type `NodeStats`.
At the end of each experiment, these statistics will be printed to screen and eventually saved to a CSV file.
It's also possible to plot the statistics with the provided Python scripts.


For your convenience, additional helper functions are provided, in order to easily create multiple nodes of a certain type.
For example you may want to define some publisher and some subscriber nodes.

The following snippet of code will create a system with 10 subscriber nodes, with names ranging from `node_0` to `node_9`, and 2 publisher nodes, with names `node_10` and `node_11` and respectively publishing on `topic_10` and `topic_11`.
Each subscriber node will subscribe to both topics.
Nodes are stored into vectors of shared pointers.
Then it is necessary to start the respective publish or subscribe tasks of the nodes.

The subscribers spin frequency is -1, i.e. they spin as fast as possible, while the publishers publish messages at 50Hz.


```
#include "performance_test/ros2/template_factory.hpp"

int n_sub = 10;
int n_pub = 2;
std::string msg_type = "1kb"
int msg_size = 0; // this is used only if msg_type is "vector"
int sub_frequency = -1;
int pub_frequency = 50;
int experiment_duration = 10; //seconds

TemplateFactory factory();

std::vector<std::shared_ptr<MultiNode>> sub_nodes = factory.create_subscribers(0, n_sub, n_pub, msg_type);

std::vector<std::shared_ptr<MultiNode>> pub_nodes = factory.create_publishers(n_sub, n_sub + n_pub, msg_type);

factory.start_subscribers(sub_nodes, sub_frequency, nullptr);

factory.start_publishers(pub_nodes, pub_frequency, experiment_duration, msg_size, nullptr);
```

**NOTE:** At the moment, the factory methods for creating subscribers and clients in a node will create all the possible ones, according to the specified number of publishers/servers.
This means that using these methods, if there are 2 publishers they will publish on 2 different topics and every subscriber node will subscribe to all the available topics.
It is possible to manually create systems with a different topology by not using the `TemplateFactory::create_subscribers` and `TemplateFactory::create_publisher` methods.


#### C++ executables

Inside the `src/ros2` directory you can find some C++ executables which, through command line options, allow to create different ROS2 systems.

These executables will use the `TemplatedMultiNode` class and the `TemplateFactory` previously described.

##### Create a process with both pubs/subs

The simplest system that you can execute is the one in which some publisher nodes and some subscriber nodes are running in a unique process.

    $ ros2 run performance_test simple_pub_sub_main --subs 5 --pubs 1 --duration 10

 - `--pubs` indicates how many nodes with a single publisher each we are going to create.
 - `--subs`  Each of them will subscribe to all the available topics.
 - `--duration` indicates how many seconds the experiment will last.

Each of these nodes will be a `TemplatedMultiNode` object.
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

There is also a variant of this function which will also monitor the CPU usage and other metrics while the process is running.

```
run_and_monitor OUTPUT_PERFORMANCE_CSV simple_pub_sub_main --subs 5 --pubs 1 --duration 10 --filename OUTPUT_CSV_FILE
```

Note that for this second function it's possible to specify two different CSV output files.
The first is actually mandatory, while the second one is optional.

**NOTE:** we are using an external script to monitor the process instead of the `getrusage` function contained in `sys/resource.h` because the latter may be not portable and does not allow to obtain instantaneous values, but only maximal ones.

## Add a new interface

**NOTE:** a simple way for testing a specific message size is to just use the message type `vector` in combination of the `msg_size` value.

 - Create your new interface in the package, placing it under `performance_test/interfaces/msg`.
   The only constraint for the interface is that it must include a field of type `std_msgs/Header` named `header`.

      performance_test/interfaces/msg/blabla.msg

      std_msgs/Header header
      bool new_field

 - Add the new interface path to the `performance_test/CMakeLists.txt` file, inside the `CUSTOM_INTERFACES` list.

      interfaces/msg/blabla.msg

 - Include the generated header file in `performance_test/src/ros2/utilities/template_factory.cpp`.

      #include performance_test/msg/blabla.hpp

 - Add a map element for creating a node templated with this interface in the `make_templated` method in `performance_test/src/ros2/utilities/template_factory.cpp`.
  The key for this map element has to be a lowercase string and this is what the user will have to provide as `--msg_type` argument in order to select this new interface.

      {"blabla", [name, this] { return std::make_shared<TemplatedMultiNode<performance_test::msg::Blabla>>(name, _ros2_namespace); } },


## Create your own experiment

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

 If you want to monitor the process performance you should substitute the `run` function with

```
run_and_monitor $PROCESS_STATS_FILE $CMD $CMD_ARGS
```

Note that `$PROCESS_STATS_FILE` has to be different from the `--filename $OUTPUT_FILE_PATH`.
The former will contain statistics about the process we are going to run (CPU, memory usage, etc) while the second will contain statistics for each ROS2 node running inside the process (latency, messages received, etc).


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