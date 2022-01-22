# ROS2 Performance Tests

This package allows to run different ROS2 systems, made of any number of nodes, and evaluate their performances.

The evaluation metrics used are

 - Latency
 - CPU usage
 - Memory usage
 - Reliability
 - Discovery time

This README describes how to setup and run the experiments.
For detailed instructions about how the framework is implemented and how it can be extended, refer to [README_dev](README_dev.md).

## Requirements

 - ROS2 SDK
 - [performance_test_msgs](../performance_test_msgs)
 - Python3 (only for plotting the results)


## Build

The performance tests can be built and run on the laptop as well as cross-compiled and run on an embedded platform as RaspberryPi.
The package has been tested for ROS2 Bouncy and Crystal releases.

#### Laptop instructions

Create a ROS2 workspace.

    mkdir -p ~/ros2_performance_ws/src

Populate the `src` directory with the `performance_test_msgs` and `performance_test` package.

    cd ~/ros2_performance_ws/src
    ln -s <path_to_this_repository>/performances/performance_test_msgs .
    ln -s <path_to_this_repository>/performances/performance_test .

Build this workspace.

    cd ~/ros2_performance_ws
    colcon build

Open with a text editor the `~/ros2_performance_ws/src/performance_test/env.sh` script and modify the following variables:

 - ROS2_SDK_INSTALL_PATH=<path_to_ros2_sdk_install>
 - ROS2_PERFORMANCE_TEST_INSTALL_PATH=~/ros2_performance_ws/install
 - MERGE_INSTALL=false

Depending on whether you installed ROS2 from sources or using the binaries, the `path_to_ros2_sdk_install` will be different.
If you installed from sources and the ROS2 SDK workspace is for example `~/ros2_ws`, then you have to set the variable to `~/ros2_ws/install`.
If you installed using the binaries and you are using the Crystal release, then you have to set the variable to `/opt/ros/crystal`.

#### Cross-compilation instructions

Create a ROS2 workspace.

    mkdir -p ~/ros2_performance_ws/src

Populate the `src` directory with the `performance_test_msgs` and `performance_test` package.

    cd ~/ros2_performance_ws/src
    cp -r <path_to_this_repository>/performances/performance_test_msgs .
    cp -r <path_to_this_repository>/performances/performance_test .

Cross-compile the workspace.

Copy the cross-compiled package to the target board.

    scp -r ~/ros2_performance_ws/install/ user@address:~/performance_ws_install

Copy the `performance_test` package to the target board.

    scp -r  ~/ros2_performance_ws/src/performance_test/ user@address:~/performance_test

On the target board, open with a text editor `~/performance_test/env.sh` and modify the following variables:

 - ROS2_SDK_INSTALL_PATH=<path_to_ros2_sdk_install>
 - ROS2_PERFORMANCE_TEST_INSTALL_PATH=~/performance_ws_install
 - MERGE_INSTALL=<true_or_false>

**Note:** set the `ROS2_SDK_INSTALL_PATH` and `MERGE_INSTALL` according to how you cross-compiled the ROS2 SDK and the performance_test package.

## Usage

#### Running experiments

Change directory to the `performance_test` package directory.

Before running any experiment you must set some environment variables.
You should have already edited the `env.sh` script with valid values for your system during the build phase.

    source env.sh

Now everything should be ready to run all the experiments that you find in the `scripts` directory.
The output will be usually displayed on screen and saved into subdirectories under `results` in form of one or more CSV files.

#### Experiment: latency and reliability

Usage:

    bash scripts/pub_sub_ros2.sh


This script runs a series of tests using publisher/subscriber systems with different parameters.

The default values of the parameters is contained in the bash script. Values can be changed exporting the proper environment variables.
For example:

    export MSG_TYPES=1mb
    export MAX_PUBLISHERS=2
    export MAX_SUBSCRIBERS=10
    export PUBLISH_FREQUENCIES=100
    export DURATION=30
    export NUM_EXPERIMENTS=5

Note that you can also pass multiple values to some variables. E.g.

    export MSG_TYPES="10kb 1mb 4mb"
    export PUBLISH_FREQUENCIES="10 50 100"

By default this script will measure latency, reliability, RAM and CPU.
You can control if you want to measure RAM and CPU using the following environment variable.

    export MON_CPU_RAM=0


**NOTE:** all the message types contains statically allocated arrays of the specified size. If you want to test a dynamically allocated message you have to set the following variables:

    export MSG_TYPES=vector
    export MSG_SIZE=10000

Where `MSG_SIZE` is the size in bytes of the dynamically allocated field. Note that this varialbes is ignored if `MSG_TYPES` is not `vector`.


For each combination of the environment variables we get:
- Latency of published messages
- Reliability of the publisher and subscriber nodes
- CPU and RAM usage during the individual test


**NOTE:** there are two additional variants of this experiment bash script.

The first one is `scripts/pub_sub_separate_process.sh`. This is basically the same script as before, but it creates publishers and subscribers in two different processes.
The arguments are exactly the same previously described, however, note that only the subscriber process will be monitored.

The second one is `scripts/client_service_ros2.sh` and, as its name states, it creates a system made of client and services.
There are some changes in the parameters, some of them are renamed and `MSG_SIZE` and `MSG_TYPES` are not present anymore.

    export MAX_SERVICES=2
    export MAX_CLIENTS=5
    export REQUEST_FREQUENCIES=100


#### Results

The output results are stored in CSV files, named as `<NUM_PUBLISHERS>p_<NUM_SUBSCRIBERS>s_<NUM_ITERATION>_<MSG_SIZE>_<PUBLISH_FREQUENCY>hz.csv`.
They are stored in directories under `results/DIR_PATH` where `DIR_PATH` by default is the name of the bash script you just run or it can be set through an environment variable.

For example if you run the following:

    export MAX_PUBLISHERS=1
    export MAX_SUBSCRIBERS=2
    export NUM_EXPERIMENTS=1
    export MSG_TYPES="10b 1kb"
    export PUBLISH_FREQUENCIES="10 100"
    export DIR_PATH=my_experiment
    bash scripts/pub_sub_ros2.sh

You will get the following output structure:

```
$ ls results/my_experiment
cpu_ram_1p_1s_10b_100hz_1.csv  lat_rel_1p_1s_10b_100hz_1.csv
cpu_ram_1p_1s_10b_10hz_1.csv   lat_rel_1p_1s_10b_10hz_1.csv
cpu_ram_1p_1s_1kb_100hz_1.csv  lat_rel_1p_1s_1kb_100hz_1.csv
cpu_ram_1p_1s_1kb_10hz_1.csv   lat_rel_1p_1s_1kb_10hz_1.csv
cpu_ram_1p_2s_10b_100hz_1.csv  lat_rel_1p_2s_10b_100hz_1.csv
cpu_ram_1p_2s_10b_10hz_1.csv   lat_rel_1p_2s_10b_10hz_1.csv
cpu_ram_1p_2s_1kb_100hz_1.csv  lat_rel_1p_2s_1kb_100hz_1.csv
cpu_ram_1p_2s_1kb_10hz_1.csv   lat_rel_1p_2s_1kb_10hz_1.csv
```

**NOTE**: by default every time you run a script all the results from its previous run are overwritten, unless you change the output directory.

    export DIR_PATH=<another directory name>


The content of a `rel_lat` csv and the one of a `cpu_ram` CSV is different.

The first one contains data where each line denotes a different ROS2 participant. For example it will contain the ID of the node, the ID of the topic/service which is used (called `communication_id`), the number of received messages, the average latency and so on.

On the other hand, the `cpu_ram` CSV contains values which are obtained from an observer which is external to the ROS2 system and thus it contains values related to the whole system.
Each line is a snapshot of the system at a specific instant of time, i.e. RAM and CPU usage. Moreover, some additional information about the ROS2 system are included (i.e. number of nodes, type of messages) which can be usesful for plotting data.
