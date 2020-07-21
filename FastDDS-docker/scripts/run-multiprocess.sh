#!/bin/sh

# Launch all nodes/processes with some time between them
# to not saturate the CPU.
# The publishers start publishing after 20 seconds.
./arequipa &
sleep 1
./barcelona &
sleep 1
./georgetown &
sleep 1
./osaka &
sleep 1
./ponce &
sleep 1
./mandalay &
sleep 1
./geneva &
sleep 1
./lyon &
sleep 1
./hamburg &
sleep 1
./montreal &

