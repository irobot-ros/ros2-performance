#!/bin/bash

SCALING_GOVERNOR=$(cat "/sys/devices/system/cpu/cpufreq/policy0/scaling_governor")

# Check if the governor is set to 'performance'
if [ "$SCALING_GOVERNOR" != "performance" ]; then
  echo "Warning: CPU scaling governor is not set to 'performance'. Current value is '$SCALING_GOVERNOR'."
  echo "run: 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'"
fi

# Get the directory where the script is located
script_dir=$(dirname "$(readlink -f "$0")")
irobot_benchmark="${script_dir}/../../../irobot_benchmark"
topologies_dir="${script_dir}/../../topologies/cli-srv"
profiles_dir="${script_dir}/../../profiles"

# Create a directory to store log folders
MP="${PWD}/cli-srv-results_multi_process/"
rm -rf $MP && mkdir -p $MP

# Define communication types
comms=("ipc_off_fastdds" "ipc_off_cyclonedds")

# Define topologies pairs
results=("10b" "100kb" "1mb" "4mb" "1mb_multi")
topology1=("cli_10b" "cli_100kb" "cli_1mb" "cli_4mb" "10_cli_1mb")
topology2=("srv_10b" "srv_100kb" "srv_1mb" "srv_4mb" "srv_1mb")

# Loop through each index in the topology1 array
for i in "${!topology1[@]}"; do
    t1=${topology1[i]}
    t2=${topology2[i]}
    res=${results[i]}

    for comm in "${comms[@]}"; do
        mkdir -p $MP/${res}/${comm}

        # Set environment variables
        if [ "$comm" == "ipc_off_fastdds" ]; then
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        else
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        fi

        # Topology
        cli_top="${topologies_dir}/${t1}.json"
        srv_top="${topologies_dir}/${t2}.json"

        # Results folder

        # Run the command
        COMMAND="${irobot_benchmark} $cli_top $srv_top -x 3 --ipc off -t 3 -s 1000 --csv-out on"
        echo -e "\nCommand: \n$COMMAND\n"
        eval $COMMAND

        # Move log folders
        mv *log $MP/${res}/${comm}
    done
done
