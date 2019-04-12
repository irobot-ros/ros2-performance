# Performances

## iRobot ROS2 performance evaluation

For each ROS2 core concept, we developed several tests for evaluating its performances.

[iRobot ROS2 Performance Evaluation Framework](performance_test)

These are the main metrics that we are interested in:

 - Latency
 - Reliability
 - CPU usage
 - Memory usage
 - Code size

Due to our specific application scenario and to the new ROS2 multithread capabilities, we made most of the evaluations running all nodes in a single process.
Evaluations have been performed on standard x86_64 laptops as well as on embedded platforms.

If you want to repeat the experiments, eventually with different parameters, follow the [Build Instructions](performance_test/README.md).

##### Motivations

The goal is to develop a set of experiments which are easily reproducible and provide useful insights on the ROS2 performances.
This allows to easily repeat them as soon as a new ROS2 distribution is released.

For each ROS2 distribution we aim to:

 - Test all the ROS2 features.
 - Find limits/bugs.
 - Perform regression tests with respect to the previous distribution.

Early results are suggesting us that this is the correct way to go, as we are spotting the strengths and weaknesses of ROS2 and helping the ROS2 community to grow, by signaling and fixing bugs to the ROS2 core implementation.

##### Experiments

Here you find links to the experiments for the most recent ROS2 distribution.
Each `README` file contains a description of the experiment, instructions for reproducing it and the results obtained.



| Experiment | README |
| ------------- | ------------- |
| Discovery time | [README.md](experiments/crystal/discovery_time) |
| Pub/Sub Latency | [README.md](experiments/crystal/pub_sub_latency) |
| Pub/Sub Reliability | [README.md](experiments/crystal/pub_sub_reliability) |
| Pub/Sub Memory usage | [README.md](experiments/crystal/pub_sub_memory) |
| Pub/Sub CPU usage | [README.md](experiments/crystal/pub_sub_cpu) |
| Client/Service systems | [README.md](experiments/crystal/client_service) |
| Bouncy-Crystal regression test | [README.md](experiments/crystal/regression) |


## External tools an resources

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
