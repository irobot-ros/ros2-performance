#!/bin/sh

CASA=$PWD
cd ../

# RMW="stub"
RMW="fastrtps"
MSG="10b 100b 250b 1kb 10kb 100kb 250kb"

for rmw in $RMW
do
    echo "RMW_IMPLEMENTATION=${rmw}"
    cd $CASA/../results/$rmw/multi-process

    for msg in $MSG
    do
        tail -1  1n_1s_${msg}_log/latency_all.txt
    done
done

cd $CASA