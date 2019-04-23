# Regression test between ROS2 Bouncy and Crystal

In the following regression tests, we compared the difference in performances between ROS2 Bouncy and Crystal.

During this process we discovered and reported several bugs, mainly in the DDS implementation, which contributed to improve the performance measures.

For this reason, in several experiments we also add results for a so called **Crystal Master** version.
These results have been obtained building the master branch of all ROS2 repositories at 03/14/2019.
These results can be replicated using the [Crystal Patch 3 version](https://github.com/ros2/ros2/tree/crystal), which has been released few days later, and only changing the FastRTPS version to release 1.7.1 .


## Latency

```
$ source env.sh
$ export MAX_PUBLISHERS=1
$ export MAX_SUBSCRIBERS=10
$ export MSG_TYPES=10b
$ export PUBLISH_FREQUENCY=100
$ export DURATION=30
$ export NUM_EXPERIMENTS=10
$ bash scripts/pub_sub_ros2.sh
```

The latency measured on RaspberryPi2 is almost the same for both Bouncy and Crystal


## Reliability

If the nodes have time to discover each other (see following `Discovery time` section), there are no noticeable changes in the reliability which is always almost 100%.


## Discovery time

OpenSplice has an almost instantaneous discovery time for nodes in the same process.
This is not changed from Bouncy to Crystal.

FastRTPS has been working a lot on improving its discovery time, with not so perfect results.

The next plot shows the average discovery time required in different types of ROS2 systems across Bouncy, Crystal first release and Crystal Master + FastRTPS v1.71 (20/02/2019).
Outliers have been excluded from the average discovery time computation, but, if present, have been added as stacked black lines.

The experiments have been run on a standard laptop, with nodes all in the same process and added to the same `SingleThreadedExecutor`.

What we can see from the plot is that, while the average discovery time in best-case scenarios (i.e. excluding outliers) has increased with successive distributions, the number of outliers is reduced a lot.

For example in Crystal first release, the best-case scenario discovery time is very good in the 10 and 15 subscribers cases with 40ms and 100ms. However we have respectively outlier experiments requiring approximately 40 seconds for discovering all the nodes in 30% and 50% of the tests.
Note that in these outliers cases, all the nodes except for 1 or 2 are immediately discovered, but the system takes ages for discovering the last ones.

Moreover with Bouncy it was not possible to run the 15 subscribers experiment as 80% of the times we reached a timeout of 100 seconds without discovering all the nodes.

On the other hand, the Crystal Master build, using FastRTPS v1.7.1, had no outliers in the 10 subscribers case and less than 10% of them in the 15 subscribers case.

A description of the changes in the FastRTPS 1.7.1 release can be found [here](https://github.com/eProsima/Fast-RTPS/issues/359).

![Plot](discovery_time_regression.png)



## Memory

Reading this old thread, dating back to April 2017, https://discourse.ros.org/t/fast-rtps-memory-usage-please-help/1739
the amount of memory used by simple nodes was reported as

 - `talker`: 4.7MB
 - `talker with 40 publishers`: 8.7MB

#### Laptop [X86_64]


| ROS2 System   | Distribution | DDS        |  Physical RAM  | Virtual RAM
| ------------- | ------------- | ------------- | ------------- | ------------- |
| publisher_lambda   | Bouncy       | FastRTPS   | 17MB           | 320MB
| publisher_lambda   | Crystal      | FastRTPS   | 20MB           | 420MB


The first thing we note is that the amounts of physical and virtual memory required has increased a lot since the Beta release.
Also between Bouncy and Crystal we can see an increment of approximately 15% in the physical memory and of approximately 30% for the virtual one.

It's interesting to note where this additional memory is used. In Bouncy, calling `rclcpp::init()` allocates 8MB of memory, entirely statically allocated. On the other hand the same function in Crystal, statically allocates 2.5MB and then dynamically allocates additional 8MB.

Moreover, we can also notice a regression in how the physical memory required scales with the number of nodes.

In Bouncy, after the first node, adding additional nodes up to 10 requires approximately 5MB per node.
After the 10th node, additional nodes cost approximately 8MB.

On the other hand, in Crystal, nodes require always the same amount of memory, approximately 10MB, until the 10th.
After this the memory requirement keeps increasing.
A 21st node requires 20MB of memory.

We also show the results obtained from the Crystal Master build, using FastRTPS v1.7.1, were this regression is mitigated.
However, the results are still worst than what we got for Bouncy.

| ROS2 System   | Distribution | DDS        |  Physical RAM  | Virtual RAM
| ------------- | ------------- | ------------- | ------------- | ------------- |
| 1 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0   | 18MB           | 400MB
| 2 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0  | 23MB           | 910MB
| 5 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0  | 39MB           | 2GB
| 10 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0  | 71MB           | 2.6GB
| 20 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0  | 152MB         | 3.2GB
| 1 node subscribing 10b topic   | Crystal      | FastRTPS 1.7.0  | 22MB           | 500MB
| 2 node subscribing 10b topic   | Crystal      | FastRTPS 1.7.0  | 31MB           | 950MB
| 5 node subscribing 10b topic   | Crystal      | FastRTPS 1.7.0  | 62MB           | 1.3GB
| 10 node subscribing 10b topic   | Crystal      | FastRTPS 1.7.0  | 135MB           | 2.1GB
| 20 node subscribing 10b topic   | Crystal      | FastRTPS 1.7.0  | 352MB           | 3.5GB
| 1 node subscribing 10b topic   | Crystal Master | FastRTPS 1.7.1  | 22MB           | 490MB
| 2 node subscribing 10b topic   | Crystal Master | FastRTPS 1.7.1  | 29MB           | 790MB
| 5 node subscribing 10b topic   | Crystal Master | FastRTPS 1.7.1  | 51MB           | 2GB
| 10 node subscribing 10b topic   | Crystal Master  | FastRTPS 1.7.1  | 104MB           | 2.7GB
| 20 node subscribing 10b topic   | Crystal Master  | FastRTPS 1.7.1  | 263MB           | 3.2GB


#### FastRTPS regression

As shown by the previous tables, there is a huge difference in the memory requirements between Bouncy and Crystal.
We did further tests and we noticed that all the additional memory is due to the different FastRTPS version which is shipped with the ROS2 code.

We created an experiment to examine the memory requirements of FastRTPS participants, without any ROS2 code involved.
The results follow the same regression shown in the previous table.

| System   | Distribution | DDS        |  Physical RAM  | Virtual RAM
| ------------- | ------------- | ------------- | ------------- | ------------- |
| 1 participants   |        | FastRTPS 1.6.0   | 9MB           | 138MB
| 2 participants   |        | FastRTPS 1.6.0  | 13MB           | 570MB
| 5 participants   |        | FastRTPS 1.6.0  | 25MB           | 1.27GB
| 10 participants   |        | FastRTPS 1.6.0  | 47MB           | 2.44GB
| 20 participants   |        | FastRTPS 1.6.0  | 99MB         | 2.84GB
| 1 participants   |       | FastRTPS 1.7.0  | 15MB           | 215MB
| 2 participants   |       | FastRTPS 1.7.0  | 20MB           | 585MB
| 5 participants   |       | FastRTPS 1.7.0  | 41MB           | 1.3GB
| 10 participants   |       | FastRTPS 1.7.0  | 81MB           | 2.5GB
| 20 participants   |       | FastRTPS 1.7.0  | 188MB           | 3.1GB
| 1 participants   |   | FastRTPS 1.7.1  | 11MB           | 206MB
| 2 participants   |   | FastRTPS 1.7.1  | 15MB           | 572MB
| 5 participants   |   | FastRTPS 1.7.1  | 26MB           | 1.27GB
| 10 participants   |    | FastRTPS 1.7.1  | 50MB           | 2.4GB
| 20 participants   |    | FastRTPS 1.7.1  | 101MB           | 2.83GB


#### RaspberryPi2

Also in the RaspberryPi2 we can notice an increase in memory from Bouncy to Crystal, with the same percentage observed in the laptop for what concerns the physical memory, but smaller for what concerns the virtual one.

However for some reason the memory usage using the Crystal Master with FastRTPS v 1.7.1 is smaller than the Bouncy one until the number of nodes is less than 20.

The plot as well as the following tables contains the same results.


| ROS2 System   | Distribution | DDS        |  Physical RAM  | Virtual RAM
| ------------- | ------------- | ------------- | ------------- | ------------- |
| 1 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0  | 11MB           | 77MB
| 2 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0   | 16MB           | 133MB
| 5 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0  | 32MB           |310MB
| 10 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0  | 60MB           | 651MB
| 20 node subscribing 10b topic   | Bouncy       | FastRTPS 1.6.0   | 135MB         | 1.5GB
| 1 node subscribing 10b topic   | Crystal       | FastRTPS 1.7.0  | 13MB           | 87MB
| 2 node subscribing 10b topic   | Crystal       | FastRTPS 1.7.0  | 18MB           | 143MB
| 5 node subscribing 10b topic   | Crystal       | FastRTPS 1.7.0   | 38MB           |324MB
| 10 node subscribing 10b topic   | Crystal       | FastRTPS 1.7.0   | 78MB           | 677MB
| 20 node subscribing 10b topic   | Crystal      | FastRTPS 1.7.0  | 216MB           | 1.56GB
| 1 node subscribing 10b topic   | Crystal Master       | FastRTPS 1.7.1  | 10MB           | 85MB
| 2 node subscribing 10b topic   | Crystal Master       | FastRTPS 1.7.1   | 12MB           | 138MB
| 5 node subscribing 10b topic   | Crystal Master       | FastRTPS 1.7.1   | 22MB           | 313MB
| 10 node subscribing 10b topic   | Crystal Master       | FastRTPS 1.7.1  | 52MB           | 653MB
| 20 node subscribing 10b topic   | Crystal Master      | FastRTPS 1.7.1  | 152MB           | 1.52GB

