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
SP="${PWD}/cli-srv-results_single_process/"
rm -rf $SP && mkdir -p $SP

# Define communication types
comms=("ipc_on" "ipc_off_fastdds" "ipc_off_cyclonedds")

topologies=(
  "cli_srv_10b"
  "cli_srv_100kb"
  "cli_srv_1mb"
  "cli_srv_4mb"
  "10_cli_srv_1mb"
)

# Loop through each combination of topology and communication type
for topology in "${topologies[@]}"; do
    mkdir -p $SP/${topology}

    for comm in "${comms[@]}"; do
        # Set environment variables
        if [ "$comm" == "ipc_off_cyclonedds" ]; then
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        else
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        fi

        if [ "$comm" == "ipc_on" ]; then
            ipc_option="--ipc on"
        else
            ipc_option="--ipc off"
        fi

        top="${topologies_dir}/${topology}.json"

        # Results folder
        result_folder=${comm}

        # Run the command
        COMMAND="${irobot_benchmark} $top -x 3 $ipc_option -t 3 -s 1000 --csv-out on --results-dir $result_folder"
        echo -e "\nCommand: \n$COMMAND\n"
        eval $COMMAND

        mv $result_folder $SP/${topology}
    done
done
