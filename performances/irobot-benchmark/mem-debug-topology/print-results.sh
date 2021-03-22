#!/bin/sh

CASA=$PWD
cd ../

# RMW="stub"
RMW="stub cyclonedds fastrtps"

for rmw in $RMW
do
    cd $CASA/../results/$rmw

    echo "" && echo "-----------------------------------------------------" &&  echo ""

    echo "RMW_IMPLEMENTATION=${rmw}"

    echo "Multiple nodes - 1pub/1sub/10b - rmw_${rmw}_cpp - 0/1/5/10/15/20"

    echo "nodes    time[ms]       cpu[%]    arena[KB]      in_use[KB]     mmap[KB]       rss[KB]        vsz[KB]"

    echo -n " 0        " && tail -1 0n_log/resources.txt
    echo -n " 1        " && tail -1 1n_1p_1s_10b_log/resources.txt
    echo -n " 5        " && tail -1 5n_1p_1s_10b_log/resources.txt
    echo -n "10        " && tail -1 10n_1p_1s_10b_log/resources.txt
    echo -n "15        " && tail -1 15n_1p_1s_10b_log/resources.txt
    echo -n "20        " && tail -1 20n_1p_1s_10b_log/resources.txt

    # echo "" && echo "-----------------------------------------------------" &&  echo ""

    # echo "Multiple nodes - 1pub/1sub/100kb - rmw_${rmw}_cpp  - 0/1/5/10/15/20"

    # echo "nodes    time[ms]       cpu[%]    arena[KB]      in_use[KB]     mmap[KB]       rss[KB]        vsz[KB]"

    # echo -n " 0    " && tail -1 0n_log/resources.txt
    # echo -n " 1    " && tail -1 1n_1p_1s_100kb_log/resources.txt
    # echo -n " 5    " && tail -1 5n_1p_1s_100kb_log/resources.txt
    # echo -n "10    " && tail -1 10n_1p_1s_100kb_log/resources.txt
    # echo -n "15    " && tail -1 15n_1p_1s_100kb_log/resources.txt
    # echo -n "20    " && tail -1 20n_1p_1s_100kb_log/resources.txt

    echo "" && echo "-----------------------------------------------------" &&  echo ""

    echo "Single node - N pub/ N sub/10b - rmw_${rmw}_cpp  - 0/1/5/10/15/200 "

    echo "nodes    time[ms]       cpu[%]    arena[KB]      in_use[KB]     mmap[KB]       rss[KB]        vsz[KB]"

    echo -n " 0       " && tail -1 1n_0p_0s_10b_log/resources.txt
    echo -n " 1       " && tail -1 1n_1p_1s_10b_log/resources.txt
    echo -n " 5       " && tail -1 1n_5p_5s_10b_log/resources.txt
    echo -n "10       " && tail -1 1n_10p_10s_10b_log/resources.txt
    echo -n "15       " && tail -1 1n_15p_15s_10b_log/resources.txt
    echo -n "20       " && tail -1 1n_20p_20s_10b_log/resources.txt

done

cd $CASA