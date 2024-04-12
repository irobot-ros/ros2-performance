#!/bin/bash

# Set governor to performance
echo "echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# Get the directory where the script is located
script_dir=$(dirname "$(readlink -f "$0")")
irobot_benchmark="${script_dir}/../../../irobot_benchmark"
topologies_dir="${script_dir}/../../topologies/pub-sub"
profiles_dir="${script_dir}/../../profiles"

# Create a directory to store log folders
SP="${PWD}/results_single_process/"
rm -rf $SP && mkdir -p $SP

# Define message sizes and communication types
comms=("ipc_on" "ipc_off" "loaned_fastdds" "loaned_cyclone")

topologies=(
  "pub_sub_10b"
  "pub_sub_100kb"
  "pub_sub_1mb"
  "pub_sub_4mb"
  "sierra_nevada_fixed_size"
  "white_mountain_fixed_size"
)

# Check if iox-roudi is running, needed for CycloneDDS zero-copy
if ! pgrep -x "iox-roudi" > /dev/null
then
    echo "CycloneDDS: iox-roudi is NOT running. Run as: ./iox-roudi -c roudi_config.toml"
fi

# Loop through each combination of topology and communication type
for topology in "${topologies[@]}"; do
    mkdir -p $SP/${topology}

    for comm in "${comms[@]}"; do
        # Set environment variables
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

        if [ "$comm" == "loaned_fastdds" ]; then
            export FASTRTPS_DEFAULT_PROFILES_FILE="${profiles_dir}/shared_memory_fastdds_dynamic_reusable.xml"
            export RMW_FASTRTPS_USE_QOS_FROM_XML=1
        fi

        # Set environment variables for "loaned" communication type
        if [ "$comm" == "loaned_cyclone" ]; then
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export CYCLONEDDS_URI="${profiles_dir}/zero-copy-shm.xml"
        fi

        # Construct the command based on communication type
        if [ "$comm" == "ipc_on" ]; then
            ipc_option="--ipc on"
        else
            ipc_option="--ipc off"
        fi

        # Set right topology for shared memory
        if [[ "$comm" == "loaned_fastdds" || "$comm" == "loaned_cyclone" ]]; then
            top="${topologies_dir}/${topology}_loaned.json"
        else
            top="${topologies_dir}/${topology}.json"
        fi

        # Results folder
        result_folder=${comm}

        # Run the command
        COMMAND="${irobot_benchmark} $top -x 3 $ipc_option -t 30 -s 1000 --csv-out on --results-dir $result_folder"
        # COMMAND="${irobot_benchmark} $top -x 3 $ipc_option -t 30 -s 1000 --csv-out on --results-dir $result_folder --timers-separate-thread on"
        echo -e "\nCommand: \n$COMMAND\n"
        eval $COMMAND

        mv $result_folder $SP/${topology}

        # Unset environment variables
        if [[ "$comm" == "loaned_fastdds" || "$comm" == "loaned_cyclone" ]]; then
            unset FASTRTPS_DEFAULT_PROFILES_FILE
            unset RMW_FASTRTPS_USE_QOS_FROM_XML
            unset CYCLONEDDS_URI
        fi
    done
done

