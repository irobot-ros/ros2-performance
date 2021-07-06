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
    mkdir -p results/$rmw
    echo "export RMW_IMPLEMENTATION=${rmw}"
    export RMW_IMPLEMENTATION=rmw_${rmw}_cpp

    for msg in $MSG
    do
        echo "Single node - 1 pub - 1 sub - ${msg} - rmw_${rmw}_cpp"

        echo "${rmw} - COMMAND: ./irobot-benchmark single-process/1n_1p_1s_${msg}.json -x 3 --tracking off -t 10"
        ./irobot-benchmark single-process/1n_1p_1s_${msg}.json -x 3 --tracking off -t 10

        mv *log results/${rmw}

    done
done

cd $CASA