# iRobot ROS 2 Performance Evaluation Framework

This repository contains executables and tools that allow to easily simulate arbitrary ROS 2 systems and then measures their performance.
The system topology can be provided at runtime using JSON files or with command line options.

The framework tracks the following metrics:
 - Latency
 - Reliability
 - CPU usage
 - Memory usage

The core of the framework is entirely developed in C++ and it has no external dependencies beside the ROS 2 core libraries.
This makes it very easy to be compiled and used on embedded platforms.
**The iRobot cross-compilation framework can be found at: https://github.com/irobot-ros/ros2-cross-compilation**.

Note that this framework is mostly meant for evaluating single process applications.
Although it is also able to measure the performance of multi-process applications, not all metrics will be available in this case.

The nodes under test currently don't perform any sort of computation while they are tested.
This means that most of the measured resource usage is due to the overhead of ROS2 communication.

## Build

The only runtime requirement is ROS 2 rolling.
The build machine requires Python 3, CMake and colcon.

```
mkdir -p ~/performance_ws/src
cd ~/performance_ws/src
git clone https://github.com/irobot-ros/ros2-performance
cd ros2-performance
git submodule update --init --recursive
cd ../..
colcon build
```

## Run

The **[irobot_benchmark](irobot_benchmark)** package contains the main application and example of graph topologies for evaluation.

```
source ~/performance_ws/install/setup.bash
cd ~/performance_ws/install/irobot_benchmark/lib/irobot_benchmark
./irobot_benchmark topology/sierra_nevada.json
```

The results will be printed to screen and also saved in the directory `./sierra_nevada_log`.

## Extending the performance framework and testing your own system

The `irobot_benchmark/topology` directory contains some examples of json files that can be used to define a system.

If you want to create your own JSON topology, follow the instructions on [how to create a new topology](performance_test_factory/create_new_topology.md).
If you want to use your custom ROS 2 message interfaces in the topology, you should look at the [performance_test_plugin_cmake](performance_test_plugin_cmake).

### Structure of the framework

 - **[performance_test](performance_test)**: this package provides the `performance_test::PerformanceNode` class, which provides API for easily adding publishers, subscriptions, clients and services and monitor the performance of the communication.
 Moreover the `performance_test::System` class allows to start multiple nodes at the same time, while ensuring that they discover each other, and to monitor the performance of the whole system.
 Moreover, this pacakge contains scripts for visualizing the performance of applications.
 - **[performance_test_msgs](performance_test_msgs)**: this package contains basic interface definitions that are directly used by the `performance_test` package to measure performance.
 - **[performance_test_factory](performance_test_factory)**: this package provides the `performance_test_factory::TemplateFactory` class that can be used to create `performance_test::PerformanceNode` objects with specific publishers and subscriptions according to some arguments provided at runtime: this can be done through json files or command line options.
 The interfaces (msg and srv) that can be used in these nodes have to be defined in the so called `performance_test_factory_plugins`.
 - **[performance_test_plugin_cmake](performance_test_plugin_cmake)**: this package provides the CMake function used to generate a factory plugin from interfaces definitions.
 - **[irobot_interfaces_plugin](irobot_interfaces_plugin)**: this package is a `performance_test_factory_plugin` that provides all the interfaces used in the iRobot system topologies.
 - **[irobot_benchmark](irobot_benchmark)**: this package provides our main benchmark application.
 This executable can load one or multiple json topologies and it creates a ROS2 system running in a specific process from each of them.
 It also contains the json topologies used for iRobot performance evaluation.
 - **[composition_benchmark](composition_benchmark)**: this package contains applications and tools that can be used to profile ROS 2 composition concepts.

## External tools and resources

#### Apex AI ROS2 Evaluation tool

ApexAI provides an alternative valid performance evaluation framework, which allows to test different type of messages.
Our implementation is inspired by their work.

 - [Github repo](https://github.com/ApexAI/performance_test)
 - [Apex AI tool ROSCon slides](https://roscon.ros.org/2018/presentations/ROSCon2018_MiddlewarePerformanceTesting.pdf)
 - [Apex AI tool ROSCon video](https://vimeo.com/293257342)

#### Other evaluation tools

 - [RealTime Working Group ReferenceSystem](https://github.com/ros-realtime/reference-system)
 - [ROS2 communication performance evaluation](https://github.com/ros2/rclcpp/issues/634)
 - [ROS2 benchmarking framework](https://github.com/piappl/ros2_benchmarking)
 - [ROS2 QOS throughput tests](https://github.com/Adlink-ROS/adlink_ros2_tools)

#### DDS Vendors advertised performance

 - [FastRTPS Latency and Throughput](https://www.eprosima.com/index.php/resources-all/performance/40-eprosima-fast-rtps-performance)
 - [FastRTPS vs ZeroMQ](https://www.eprosima.com/index.php/resources-all/performance/zmq-vs-eprosima-fast-rtps)
 - [Connext Micro Heap usage](https://community.rti.com/static/documentation/connext-micro/2.4.10/doc/html/group__datasheet__armv6leLinux2__6gcc4__6__3.html#armv6leLinux2_6gcc4_6_3_HEAP)

#### Performances discussions

 - [Evaluation of ROS2 Communication Layer](https://roscon.ros.org/2016/presentations/rafal.kozik-ros2evaluation.pdf)
 - [ROS2 latency and throughput](https://discourse.ros.org/t/latency-and-throughput-in-ros2/4367)
 - [Evaluating QoS performances using ROS demos](https://github.com/ros2/rmw_fastrtps/issues/202)


#### Papers

 - [Exploring the performance of ROS2](https://www.semanticscholar.org/paper/Exploring-the-performance-of-ROS2-Maruyama-Kato/07b895f3b584dea4f64e91844f243de382026b20)
 - [Time-Sensitive Networking for robotics](https://arxiv.org/abs/1804.07643)
 - [Real-time Linux communications: an evaluation of the Linux communication stack for real-time robotic applications](https://arxiv.org/pdf/1808.10821.pdf)
 - [Towards a distributed and real-time framework for robots: Evaluation
of ROS 2.0 communications for real-time robotic applications](https://arxiv.org/pdf/1809.02595.pdf)
 - [Time Synchronization in modular collaborative robots](https://arxiv.org/pdf/1809.07295.pdf)
 - [Reduce delay and jitter in wireless ROS1 networks](https://arxiv.org/pdf/1707.07540.pdf)
