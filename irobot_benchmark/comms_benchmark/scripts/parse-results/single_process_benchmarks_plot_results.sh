#!/bin/bash

# Get the directory where the script is located
script_dir=$(dirname "$(readlink -f "$0")")
scripts="${script_dir}/../../scripts"

topologies=("pub_sub_10b" "pub_sub_100kb" "pub_sub_1mb" "pub_sub_4mb" "sierra_nevada_fixed_size" "white_mountain_fixed_size")
comms=("ipc_on" "ipc_off" "loaned")

for topology in "${topologies[@]}"; do
    python_args=()
    for comm in "${comms[@]}"; do
        python_args+=("../results_single_process/${topology}/${comm}")
    done
    python3 $scripts/plot_cpu.py "${python_args[@]}"
    python3 $scripts/plot_latency.py "${python_args[@]}"
done
