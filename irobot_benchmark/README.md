# Benchmark Application

This folder contains a benchmark application to test the performance of a ROS2 system.

To run the benchmark, supply a .json topology file, specifying a complete ROS2 system, to `irobot_benchmark`.  The application will load the complete ROS2 system from the topology file and will begin passing messages between the different nodes.

For the duration of the test, statistical data will be collected, the usage of resources (CPU utilization and RAM consumption) and message latencies.

After the user-specified duration of time, the application will output the results as human-readable, space-delimited log files.
Comma-delimited output is also available, specified via a flag to `irobot_benchmark`

### Topologies

Multiple topologies are provided in the [topology](topology) folder. Two examples are [Sierra Nevada](topology/sierra_nevada.pdf) and [Mont Blanc](topology/mont_blanc.pdf).
Sierra Nevada is light 10-node system while Mont Blanc is a heavier and more complex 20-node system.


### Usage

Follow the instructions for building the [performance_test](../performance_test) framework.

First, source the environment:

```
source performances_ws/install/local_setup.bash
```

Example run:

```
cd performances_ws/install/lib/irobot_benchmark
./irobot_benchmark topology/sierra_nevada.json -t 60 --ipc on
```

This will run Sierra Nevada for 60 seconds and with *Intra-Process-Communication* activated.
For more options, run `./irobot_benchmark --help`.

### Output

After running the application, a folder will be created in the current working directory along with four different files inside it:
- `latency_all.txt`
- `latency_total.txt`
- `resources.txt`
- `events.txt`

### Benchmark results

The following are sample files that have been obtained running Sierra Nevada on a RaspberryPi 3.


**latency_all.txt**:
```
node           topic          size[b]   received[#]    late[#]   too_late[#]    lost[#]   mean[us]  sd[us]    min[us]   max[us]   freq[hz]  duration[s]
lyon           amazon         36        12001          11        0              0         602       145       345       4300      100       120
hamburg        danube         8         12001          15        0              0         796       233       362       5722      100       120
hamburg        ganges         16        12001          10        0              0         557       119       302       4729      100       120
hamburg        nile           16        12001          18        0              0         658       206       300       5258      100       120
hamburg        tigris         16        12000          17        0              0         736       225       310       5994      100       120
osaka          parana         12        12001          32        0              0         636       236       346       4343      100       120
mandalay       danube         8         12001          16        0              0         791       189       418       6991      100       120
mandalay       salween        48        1201           1         0              0         663       297       391       6911      10        120
ponce          danube         8         12001          15        0              0         882       203       437       7270      100       120
ponce          missouri       10000     1201           0         0              0         881       245       434       3664      10        120
ponce          volga          8         241            0         0              0         954       586       413       4010      2         120
barcelona      mekong         100       241            0         0              0         844       297       425       2074      2         120
georgetown     lena           50        1201           1         0              0         707       302       368       8392      10        120
geneva         congo          16        1201           1         0              0         691       298       353       7218      10        120
geneva         danube         8         12001          26        0              0         1008      227       480       7025      100       120
geneva         parana         12        12001          40        0              0         760       275       368       4351      100       120
arequipa       arkansas       16        1201           1         2              0         810       1079      379       37064     10        120
```

**latency_total.txt**:
```
received[#]    mean[us]  late[#]   late[%]   too_late[#]    too_late[%]    lost[#]   lost[%]
126496         744       204       0.1613    2              0.001581       0         0
```

There are different message classifications depending on their latency.
* A message is classified as **too_late** when its latency is greater than `min(period, 50ms)`, where `period` is the publishing period of that particular topic.
* A message is classified as **late** if it's not classified as **too_late** but its latency is greater than `min(0.2*period, 5ms)`.
* The idea is that a real system could still work with a few **late** messages but not **too_late** messages.
* Note that there are cli options to change these thresholds (for more info: `./irobot_benchmark --help`).
* A **lost** message is a message that never arrived.
    * A lost message is detected when the subscriber receives a message with a tracking number greater than the one expected.
    * The assumption here is that the messages always arrive in chronological order, i.e., a message A sent before a message B will either arrive before B or get lost, but will never arrive after B.
* The rest of the messages are classified as **on_time**.

```
Message classifications by their latency


+                               +                               +
|                               |                               |
|                               |                               |
|                               |                               |
|                               |                               |
|                               |                               |
+-------------------------------+-------------------------------+

<--------><---------------------><----------------------------------->
 on_time           late                          too_late

<------------------------------->
             period
```

**resources.txt** (trimmed):
```
time[ms]       cpu[%]    arena[KB]      in_use[KB]     mmap[KB]       rss[KB]        vsz[KB]
0              0         0              0              0              0              0
500            27        61080          60712          0              24948          305212
1000           51        167860         167539         0              47060          544012
1500           50        202844         202753         0              55496          660724
2000           43        202860         202769         0              55496          660724
2500           39        202876         202786         0              55496          660724
3000           36        202900         202801         0              55496          660724
3500           34        202928         202822         0              55496          660724
4000           33        202936         202837         0              55496          660724
4500           31        202948         202853         0              55496          660724
5000           31        202968         202868         0              55496          660724
5500           30        202992         202893         0              55496          660724
6000           29        203012         202909         0              55496          660724
6500           29        203028         202925         0              55496          660724
7000           28        203036         202940         0              55496          660724
7500           28        203052         202955         0              55496          660724
8000           28        203068         202971         0              55496          660724
8500           27        203092         202986         0              55496          660724
9000           27        203104         203001         0              55496          660724
9500           27        203140         203038         0              55496          660724
10000          27        203280         203054         0              55792          661748
10500          26        203284         203069         0              55792          661748
```

* The CPU and Memory resources are sampled every 0.5 seconds (can be changed with the option `--sampling`).
* CPU percent utilization is measured over the total cores, i.e., a 100% CPU utilization on a 4-core platform means that all 4 cores are 100% busy.  More specifically:
  * `time_spent_in_program`: calls `clock()` which returns the amount of time spent in our program, e.g. (now_clock - start_clock) / `CLOCKS_PER_SEC`
  * `total_threaded_duration`: current duration * `num_threads`
  * `result = (time_spent_in_program / total_threaded_duration) * 100`
* The fields **arena**, **in_use** (uordblks) and **mmap** (hblkhd) are obtained by calling [mallinfo](http://man7.org/linux/man-pages/man3/mallinfo.3.html).
  * These fields represent the total memory allocated by the `sbrk()` and `mmap()` system calls.
* The field **rss** is the actual allocated memory that was mapped into physical memory.
  * Note that an allocated memory page is not mapped into physical memory until the executing process demands it ([demand paging](https://en.wikipedia.org/wiki/Demand_paging)).
* **vsz** represents the size of the virtual memory space.

For our benchmark, **rss** is the most important memory metric.

**events.txt** (trimmed):
```
Time[ms]    Caller                   Code  Description
90          SYSTEM                   0     [discovery] PDP completed
151         SYSTEM                   0     [discovery] EDP completed
156         amazon->lyon             1     msg 0 late. 4300us > 2000us
156         danube->mandalay         1     msg 0 late. 4082us > 2000us
250         danube->ponce            1     msg 10 late. 2303us > 2000us
250         danube->geneva           1     msg 10 late. 2617us > 2000us
338         arkansas->arequipa       2     msg 0 too late. 184382us > 50000us
338         arkansas->arequipa       2     msg 1 too late. 136936us > 50000us
339         arkansas->arequipa       1     msg 2 late. 37064us > 5000us
2314        parana->geneva           1     msg 216 late. 2097us > 2000us
2314        parana->osaka            1     msg 216 late. 2131us > 2000us
3614        parana->osaka            1     msg 346 late. 2044us > 2000us
3615        parana->geneva           1     msg 346 late. 2682us > 2000us
3644        nile->hamburg            1     msg 349 late. 2234us > 2000us
3645        tigris->hamburg          1     msg 349 late. 2081us > 2000us
4650        danube->mandalay         1     msg 450 late. 2432us > 2000us
5145        parana->osaka            1     msg 499 late. 3243us > 2000us
5149        danube->ponce            1     msg 500 late. 2120us > 2000us
5149        danube->geneva           1     msg 500 late. 2140us > 2000us
5155        parana->osaka            1     msg 500 late. 2440us > 2000us
5155        parana->geneva           1     msg 500 late. 2536us > 2000us
5784        parana->geneva           1     msg 563 late. 2157us > 2000us
6647        ganges->hamburg          1     msg 650 late. 2302us > 2000us
7004        parana->geneva           1     msg 685 late. 2172us > 2000us
7004        parana->osaka            1     msg 685 late. 2138us > 2000us
8147        tigris->hamburg          1     msg 799 late. 4431us > 2000us
8148        ganges->hamburg          1     msg 800 late. 3775us > 2000us
11149       danube->mandalay         1     msg 1100 late. 2019us > 2000us
11150       danube->hamburg          1     msg 1100 late. 2736us > 2000us
12650       danube->hamburg          1     msg 1250 late. 2314us > 2000us
12650       danube->geneva           1     msg 1250 late. 2897us > 2000us
```

This file stores special events with their associated timestamp, such as:
- late message
- too late message
- lost message
- system nodes discovery

### Target performance

The target performance for different topologies on specific platforms can be found in the folder [performance_target](performance_target).
For example, [sierra_nevada_rpi3.json](performance_target/sierra_nevada_rpi3.json):

```
{
    "topology_file": "sierra_nevada.json",
    "platform": "rpi3 b 1.2",
    "additional_options": "-t 600 --ipc on -s 1000 --late-percentage 20 --late-absolute 5000 --too-late-percentage 100 --too-late-absolute 50000",
    "comments": "scaling governor should be set to 'performance' at 800MHz",
    "resources": {
        "cpu[%]":   15,
        "rss[KB]":  10240
    },
    "latency_total": {
        "late[%]":      1.9,
        "too_late[%]":  0.1,
        "lost[%]":      0.0
    }
}
```

### Plotting

After you have run the application, you can plot the results using the plot scripts described in the [performance_test](../performance_test) library.

Moreover, it's possible to directly compare the results with a performance target defined in a *.json* file.
For example, you can run:

```
python3 <path_to_performance_test_pkg>/scripts/visualization/benchmark_app_evaluation.py --target <path_to_benchmark_pkg>/performance_target/sierra_nevada_rpi3.json --resources log/resources.txt --latency log/latency_total.txt
```

Also, you can use the performance target *.json* file together with the `cpu_ram_plot.py` script

```
python3 <path_to_performance_test_pkg>/scripts/visualization/cpu_ram_plot.py log/resources.txt --x time --y cpu --y2 rss --target <path_to_benchmark_pkg>/perf_target.json
```

### Results

For reference only, these are the results obtained by running the default topologies on an RPi3 using ROS2 Dashing.

#### Sierra Nevada
![Plot](sierra_nevada_bar_plot.png)

#### Mont Blanc
![Plot](mont_blanc_bar_plot.png)
