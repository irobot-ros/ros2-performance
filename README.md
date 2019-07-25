# Performances

## iRobot ROS2 performance evaluation framework

[ROS2 Performance Framework](performances)

We developed a framework that allows to easily create arbitrary ROS2 systems and then measures their performance.
With this framework you can define your system at runtime using json files or with command line options.

These are the main metrics that we are interested in:

 - Latency
 - Reliability
 - CPU usage
 - Memory usage

Due to our specific application scenario and to the new ROS2 multithread capabilities, we made most of the evaluations running all nodes in a single process.
However, our frameworks also support having multiple processes running at the same time.

The goal is to develop a set of experiments which are easily reproducible and provide useful insights on the ROS2 performances.
Moreover, it is possible to create proxy applications that mimic a real, complex, system and to measure what's the overhead of ROS2 communication in such a context.

We can easily repeat our tests with different ROS2 distributions or middlewares.
For each ROS2 distribution we aim to:

Early results are suggesting us that this is the correct way to go, as we are spotting the strengths and weaknesses of ROS2 and helping the ROS2 community to grow, by signaling and fixing bugs to the ROS2 core implementation.
Running our application also allowed to find regressions with respect to the previous releases.


## Proposals

[Proposals](proposals)

This directory contains the design proposals that we made to improve the ROS2 core and its performances.

## Cross-Compilation

[Cross-Compilation](cross-compiling)

This repository also contains a Docker-based cross-compilation framework for cross-compiling the whole ROS2 SDK or single packages for the RaspberryPi.
This framework has been designed with the objective in mind of being easy to be extended to new target architectures.

You can find detailed instructions in the [cross-compiling directory](cross-compiling).


## External tools and resources

#### Apex AI ROS2 Evaluation tool

ApexAI provides an alternative valid performance evaluation framework, which allows to test different type of messages.
Our implementation is inspired by their work.

 - [Github repo](https://github.com/ApexAI/performance_test)
 - [Apex AI tool ROSCon slides](https://roscon.ros.org/2018/presentations/ROSCon2018_MiddlewarePerformanceTesting.pdf)
 - [Apex AI tool ROSCon video](https://vimeo.com/293257342)

#### Other evaluation tools

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
