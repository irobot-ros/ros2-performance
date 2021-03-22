#!/bin/sh

CASA=$PWD
cd ../
rm -r *log
rm -r results


# RMW="stub"
RMW="stub cyclonedds fastrtps"

for rmw in $RMW
do
    mkdir -p results/$rmw

    echo "export RMW_IMPLEMENTATION=${rmw}"

    export RMW_IMPLEMENTATION=rmw_${rmw}_cpp

    echo "Multiple nodes - 1 pub - 1 sub - 10b - rmw_${rmw}_cpp"

    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/0n.json -x 1 --tracking off -t 1"
    ./irobot-benchmark mem-debug-topology/0n.json -x 1 --tracking off -t 1
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_1p_1s_10b.json -x 1 --tracking off -t 1"
    ./irobot-benchmark mem-debug-topology/1n_1p_1s_10b.json -x 1 --tracking off -t 1
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/5n_1p_1s_10b.json -x 1 --tracking off -t 1"
    ./irobot-benchmark mem-debug-topology/5n_1p_1s_10b.json -x 1 --tracking off -t 1
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/10n_1p_1s_10b.json -x 1 --tracking off -t 2"
    ./irobot-benchmark mem-debug-topology/10n_1p_1s_10b.json -x 1 --tracking off -t 2
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/15n_1p_1s_10b.json -x 1 --tracking off -t 3"
    ./irobot-benchmark mem-debug-topology/15n_1p_1s_10b.json -x 1 --tracking off -t 3
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/20n_1p_1s_10b.json -x 1 --tracking off -t 4"
    ./irobot-benchmark mem-debug-topology/20n_1p_1s_10b.json -x 1 --tracking off -t 4

    echo "" && echo "-----------------------------------------------------" &&  echo ""

    # echo "Multiple nodes - 1 pub - 1 sub - 100kb - rmw_${rmw}_cpp"
    # echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_1p_1s_100kb.json -x 1 --tracking off -t 1"
    # ./irobot-benchmark mem-debug-topology/1n_1p_1s_100kb.json -x 1 --tracking off -t 1
    # echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/5n_1p_1s_100kb.json -x 1 --tracking off -t 1"
    # ./irobot-benchmark mem-debug-topology/5n_1p_1s_100kb.json -x 1 --tracking off -t 1
    # echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/10n_1p_1s_100kb.json -x 1 --tracking off -t 2"
    # ./irobot-benchmark mem-debug-topology/10n_1p_1s_100kb.json -x 1 --tracking off -t 2
    # echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/15n_1p_1s_100kb.json -x 1 --tracking off -t 3"
    # ./irobot-benchmark mem-debug-topology/15n_1p_1s_100kb.json -x 1 --tracking off -t 3
    # echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/20n_1p_1s_100kb.json -x 1 --tracking off -t 4"
    # ./irobot-benchmark mem-debug-topology/20n_1p_1s_100kb.json -x 1 --tracking off -t 4
    # echo "" && echo "-----------------------------------------------------" &&  echo ""

    echo "Multiple nodes - 1 pub - 1 sub - 100kb - rmw_${rmw}_cpp"

    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_0p_0s_10b.json -x 1 --tracking off -t 1"
    ./irobot-benchmark mem-debug-topology/1n_0p_0s_10b.json -x 1 --tracking off -t 1
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_1p_1s_10b.json -x 1 --tracking off -t 1"
    ./irobot-benchmark mem-debug-topology/1n_1p_1s_10b.json -x 1 --tracking off -t 1
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_5p_5s_10b.json -x 1 --tracking off -t 1"
    ./irobot-benchmark mem-debug-topology/1n_5p_5s_10b.json -x 1 --tracking off -t 1
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_10p_10s_10b.json -x 1 --tracking off -t 2"
    ./irobot-benchmark mem-debug-topology/1n_10p_10s_10b.json -x 1 --tracking off -t 2
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_15p_15s_10b.json -x 1 --tracking off -t 3"
    ./irobot-benchmark mem-debug-topology/1n_15p_15s_10b.json -x 1 --tracking off -t 3
    echo "${rmw} - COMMAND: ./irobot-benchmark mem-debug-topology/1n_20p_20s_10b.json -x 1 --tracking off -t 4"
    ./irobot-benchmark mem-debug-topology/1n_20p_20s_10b.json -x 1 --tracking off -t 4

    echo "" && echo "-----------------------------------------------------" &&  echo ""

    mv *log results/${rmw}

done

cd $CASA