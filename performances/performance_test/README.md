# ROS2 Performance Test

## Create a sample application

This package defines the `performance_test::Node` class.
This class inherits from `rclcpp::Node` and provides APIs for easily adding any number of publishers, subscriptions, clients and servers to the node.

Each of the `Node::add_periodic_publisher`, `Node::add_subscriber`, etc.. methods are based on a template in order to allow the creation of systems with different types of messages.

We use the `performance_test::Topic<MsgType>` templated structures in order to store the message type and the name of topics.
There exists an equivalent structure for services `performance_test::Service<SrvType>`.

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
System ros2_system();
ros2_system.add_node(pub_node);
ros2_system.add_node(sub_node);
ros2_system.spin(experiment_duration_sec);
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

## Visualizing the results

This repository contains Python scripts useful for plotting and aggregating data from multiple CSV files.
You can find them under `scripts/plot_scripts`.

Note that these scripts require `Python3` and should be run on your laptop once you have copied there the experiment results from the embedded platform.

There are two different types of CSV currently produced by our experiments. For this reason we have two different scripts to plot them.

- Latency and reliability: `scripts/plot_scripts/latency_reliability_plot.py`

- CPU and Memory: `scripts/plot_scripts/cpu_ram_plot.py`

The two scripts share most of the code, located under `scripts/plot_scripts/plot_common.py`. So let's first describe some common features.

You have to use command line arguments to tell the script which CSV files you want to load and which data you want to plot.

For a full description of the command line options, as well as an up-to-date list of the accpted values, use

    python3 scripts/plot_scripts/cpu_ram_plot.py --help

 - **dir_paths**: This is a mandatory positional argument which requires one or more paths to the CSV files. Note that it is possible to pass paths of files, directories or a mixture of them.
 - **x**: the metric to show on the X axis.
 - **y**: the metric(s) to show on the Y axis.
 - **y2**: the metric(s) to show on the secon Y axis (optional).
 - **separator**: a value according to which you want to separate your data creating different plot lines on the same axes (optional).

Starting from the easy things, you may want to plot the average CPU usage of running a ROS2 system.

    python3 scripts/plot_scripts/cpu_ram_plot.py path_to_a_csv_file --x time --y cpu

You can add a second metric on a second Y axis using the `--y2` option. For example you may want to check also the pyhsical memory usage

    python3 scripts/plot_scripts/cpu_ram_plot.py path_to_a_csv_file --x time --y cpu --y2 rss

It is possible to have more than a single value plotted against the same axis. For example you may want to check both virtual as well as physical memory usage. Note that this is just an example, the two metrics should have values with a similar magnitude or the resulting plot will be difficult to understand.

    python3 scripts/plot_scripts/cpu_ram_plot.py path_to_a_csv_file --x time --y cpu --y2 rss vsz

In all these examples, you could have also specified more than a single CSV file. The results would have been averaged.

    python3 scripts/plot_scripts/cpu_ram_plot.py path_to_csv_file1 path_to_csv_file2 --x time --y cpu --y2 rss vsz

Now let's assume that you want to compare data coming from different experiments, i.e. for different values of number of nodes, frequencies or message sizes.
This can be done in the same plot using the `--separator` option.

    export MAX_PUBLISHERS=1
    export MAX_SUBSCRIBERS=5
    export NUM_EXPERIMENTS=5
    export MSG_TYPES=10b
    export PUBLISH_FREQUENCIES=100
    export DIR_PATH=my_experiment
    bash scripts/pub_sub_ros2.sh

    python3 scripts/plot_scripts/cpu_ram_plot.py results/my_experiment/cpu_ram_* --x time --y rss --separator subs

The output will be a plot with 5 different "separated" lines, one for each possible number of subscribers. Each line will be the average of the 5 requested experiments.

Sometimes you may want to compare the content of different experiments directories, for example run with different DDS or ROS2 distributions.
This can still be done using the `--separator` option and setting it to `directory`. This means that csv files will be divided according to the directory in which they are stored.

    python3 scripts/plot_scripts/cpu_ram_plot.py results/my_experiment1/cpu_ram_* results/my_experiment2/cpu_ram_* --x time --y rss --separator directory

You can also specify more than one value for the `--separator` option, however, the resulting plot may become quite clogged.

**NOTE:** The `--x time` value is the only one which is meaningful to use if you want to plot a single csv file.
This value can be used only with the `scripts/plot_scripts/cpu_ram_plot.py` script because latency and reliability are measured once for each node at the end of the execution so you don't have instantaneous values.

**NOTE:** When using the `scripts/plot_scripts/cpu_ram_plot.py` script, if `--x` is set to something different than `time`, you will get only 1 value out of each CSV, i.e. the average of all the lines. Otherwise you will get one value for each line.


Some examples for using the `scripts/plot_scripts/latency_reliability_plot.py` script.

Plot the average latency for different numbers of subscriber nodes

    python3 scripts/plot_scripts/latency_reliability_plot.py path_to_a_csv_directory --x subs --y latency

Separate the values according to the number of publishers

    python3 scripts/plot_scripts/latency_reliability_plot.py path_to_a_csv_directory --x subs --y latency --separator pubs




