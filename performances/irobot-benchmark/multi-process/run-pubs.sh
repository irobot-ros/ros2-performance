#!/bin/sh

CASA=$PWD
cd ../
rm -r *log
rm -r results


# RMW="stub"
RMW="fastrtps"
MSG="10b 100b 250b 1kb 10kb 100kb 250kb"

for rmw in $RMW
do
    mkdir -p results/$rmw/multi-process
    echo "export RMW_IMPLEMENTATION=${rmw}"
    export RMW_IMPLEMENTATION=rmw_${rmw}_cpp

    for msg in $MSG
    do
        echo "Single node - 1 pub - ${msg} - rmw_${rmw}_cpp"

        echo "${rmw} - COMMAND: ./irobot-benchmark multi-process/1n_1p_${msg}.json -x 1 --tracking off -t 30 --ros_params off"
        ./irobot-benchmark multi-process/1n_1p_${msg}.json -x 1 --tracking off -t 30 --ros_params off

        rm -rf 1n_1p_${msg}_log
    done
done

cd $CASA