# Build and run docker with FastDDS dependencies among some debug tools
bash build.sh
bash run.sh

# Workspaces
SingleProcess_SierraNevada and MultiProcess_SierraNevada workspaces
can be compiled in the docker container (they are mounted as volumes)
or can be copied to the ROS2 SDK for cross-compilation to other architectures.

# Single Process Sierra Nevada

Test: 100 seconds
1 Participant
13 Topics
13 Publishers
17 Subscribers

# Multi Process Sierra Nevada

Under scripts folder, run-multiprocess.sh can be run to launch all nodes.

Test: 300 seconds
10 Nodes / Processes
1 Participant per node

13 Topics
13 Publishers
17 Subscribers
