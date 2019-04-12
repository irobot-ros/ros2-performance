# Publisher/Subscriber reliability

Nodes discovery phase plays an important role in the computation of the reliability of a publisher/subscriber system.

Sometimes you may see that a subscriber has not received all the messages, but, with a deeper analysis, you may notice that only the first N messages are missing.
This was most probably due to the subscriber node not being discovered in time by the publisher.



### Best-effort vs reliable mode

There are almost no differences between using reliable or best-effort mode considering a single process application with 1 publisher and a variable number of subscribers.


```
$ source env.sh
$ export MAX_PUBLISHERS=1
$ export MAX_SUBSCRIBERS=10
$ export MSG_TYPES=10b
$ export PUBLISH_FREQUENCY=100
$ export DURATION=30
$ export NUM_EXPERIMENTS=10
$ bash scripts/pub_sub_ros2.sh
$ python scripts/plot_scripts/latency_reliability_plot.py <path_to_experiments> --x subs --y reliability_sub
```

Performing the experiment both on `X86_64` as well as `RaspberryPi2`, the reliability is always 100% for both reliability modes.


### Different processes reliability

In this experiment we want to measure what's the reliability when running ROS2 nodes in different processes.

```
$ source env.sh
$ export MAX_PUBLISHERS=1
$ export MAX_SUBSCRIBERS=10
$ export MSG_TYPES=10b
$ export PUBLISH_FREQUENCY=100
$ export DURATION=30
$ export NUM_EXPERIMENTS=10
$ bash scripts/pub_sub_separate_process.sh
$ python scripts/plot_scripts/latency_reliability_plot.py <path_to_experiments> --x subs --y reliability_sub
```

We performed this test both in reliable as well as in best effort mode.
Performing the experiment both on `X86_64` as well as `RaspberryPi2`, the reliability is always 100% for both reliability modes.



### Maximum publish frequencies

We can use the reliability tests, and the count of messages sent, to measure what is the maximum frequency at which a publisher is able to send messages.

This can be achieved requesting a very high publish frequency and then seeing how many messages the publisher has been able to send.

The idea is that the requested publish frequency should exceed the limit of what the system can do and this allows to get the maximum value in each situation.


```
$ source env.sh
$ export MAX_PUBLISHERS=1
$ export MAX_SUBSCRIBERS=5
$ export MSG_TYPES="10b 100b 1kb 10kb 100kb 250kb 1mb 4mb"
$ export PUBLISH_FREQUENCY=2000
$ export DURATION=30
$ export NUM_EXPERIMENTS=10
$ bash scripts/pub_sub_ros2.sh
$ python scripts/plot_scripts/latency_reliability_plot.py <path_to_experiments> --x msg_size --y max_frequency --separator subs
```

In the following plot we also added a line for the case with 1 publisher and 0 subscribers.


Moreover the maximum reachable frequency is affected by the number of subscriptions to a topic.

**NOTE: even at the maximum frequency, the reliability was 100%.**

