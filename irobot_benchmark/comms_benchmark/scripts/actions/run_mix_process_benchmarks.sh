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
topologies_dir="${script_dir}/../../topologies/actions"
profiles_dir="${script_dir}/../../profiles"

# Create a directory to store log folders
MP="${PWD}/actions-results_mix_process/"
rm -rf $MP && mkdir -p $MP

# Define message sizes and communication types
comms=("ipc_off_fastdds" "ipc_off_cyclonedds")

# Name of the results folders
results=("10b" "100kb" "1mb" "4mb")

# Case 1: Define topologies pairs
# topology1=("srv_10b" "srv_100kb" "srv_1mb" "srv_4mb")
# Somehow an external host should synchronize and run also "remote_topology"
# The script "run_single_process_benchmarks could be of help for that"
# remote_topology=("cli_10b" "cli_100kb" "cli_1mb" "cli_4mb")


# Case 2: Define topologies pairs
topology1=("cli_srv_10b" "cli_srv_100kb" "cli_srv_1mb" "cli_srv_4mb")
topology2=("cli_10b" "cli_100kb" "cli_1mb" "cli_4mb")
# remote_topology=("cli_10b" "cli_100kb" "cli_1mb" "cli_4mb")


# Loop through each index in the topology1 array
for i in "${!topology1[@]}"; do
    t1=${topology1[i]}
    t2=${topology2[i]}
    res=${results[i]}

    for comm in "${comms[@]}"; do
        result_folder="$MP/${res}/${comm}"
        mkdir -p $result_folder

        # Set environment variables
        if [ "$comm" == "ipc_off_fastdds" ]; then
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        else
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        fi

        cli_srv="${topologies_dir}/${t1}.json"
        only_client="${topologies_dir}/${t2}.json"

        # Run the command
        COMMAND="${irobot_benchmark} $cli_srv $only_client -x 3 --ipc off -t 3 -s 1000 --csv-out on"
        echo -e "\nCommand: \n$COMMAND\n"
        eval $COMMAND

        # Move log folders
        mv *log $result_folder

        # Unset environment variables after running "loaned_fastdds" command
        if [[ "$comm" == "loaned_fastdds" || "$comm" == "loaned_cyclone" ]]; then
            unset FASTRTPS_DEFAULT_PROFILES_FILE
            unset RMW_FASTRTPS_USE_QOS_FROM_XML
            unset CYCLONEDDS_URI
        fi
    done
done
