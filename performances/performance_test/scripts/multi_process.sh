#!/bin/bash

# Script for running benchmark + debug_node
# Usage: bash multi_process.sh -h

# Parse input arguments
if [[ $# -eq 0 ]]; then set -- "-h"; fi

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -t|--topologies_list)
    TOPOLOGY_LIST="$2"

    while [[ "${3::1}" != '-' ]]
    do
        shift
        TOPOLOGY_LIST="$TOPOLOGY_LIST $2"
    done

    printf "\nTOPOLOGY_LIST: $TOPOLOGY_LIST\n"
    shift 2;;

    # IPC mode
    --ipc)
    IPC="$2"
    printf "IPC: $IPC\n"
    shift 2;;

    --dds)
    DDS="$2"
    printf "DDS: $DDS\n"
    shift 2;;

    # Duration of the test
    --time)
    TIME="$2"
    printf "TIME: $TIME seconds\n"
    shift 2;;

    # IP address of remote platform which runs the benchmark
    --ip)
    IP_ADDR="$2"
    printf "IP_ADDR: $IP_ADDR\n"
    shift 2;;

    # Location running benchmark: Local or remote
    -l|--location)
    LOCATION="$2"
    printf "LOCATION: $LOCATION\n"
    shift 2;;

    # Help
    -h|--help)
    printf "\nDescription: Script for running multiple topologies simultaneously, on single or multiple platforms.\n"
    printf "\nOptions:\n[-t|--topologies_list]\n[--ipc]:[on, off]\n"
    printf "[--dds]:[fastrtps, cyclonedds, dps]\n[--time]:[test duration in seconds]\n[--ip]\n"
    printf "\nSingle platform usage example:\nbash multi_process.sh -t sierra_nevada.json debug_sierra_nevada_reliable.json"
    printf " --ipc on --dds fastrtps --time 5\n"
    printf "\nMulti platform usage example: (use the IP address of your local device)\n"
    printf "* Local:\nbash multi_process.sh -t sierra_nevada.json --ipc on --dds fastrtps --time 5 -l local --ip 192.168.1.218\n\n"
    printf "* Remote:\nbash multi_process.sh -t debug_sierra_nevada_reliable.json --ipc on --dds fastrtps --time 5 -l remote --ip 192.168.1.218\n\n"
    exit
esac
done

if [ -z "$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH" ]; then
  printf 'Error: env.sh has not been sourced!\n' >&2
  exit 1
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
UTILITIES_DIR=$THIS_DIR/utility_scripts

# trap ctrl + c to kill all subprocesses
source $UTILITIES_DIR/kill_all_subprocesses.sh

BENCHMARK_PATH=$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH
TOPOLOGY_PATH="$THIS_DIR/../../benchmark/topology"

# Use selected RMW
export RMW_IMPLEMENTATION=rmw_"$DDS"_cpp

for TOPOLOGY in $TOPOLOGY_LIST
do
    printf "\nRunning benchmark $TOPOLOGY in a background process.\n"
    TOPOLOGY_NAME=${TOPOLOGY::-5}_log

    if [[ "$LOCATION" == "local" ]]
    then
        printf "\nWaiting for remote benchmark to start...\n"
        nc -l $IP_ADDR -p 1234

    elif [[ "$LOCATION" == "remote" ]]
    then
        SYNC_CMD="printf 'Remote device says: Start benchmark' | nc -q 1 $IP_ADDR 1234"
        until eval $SYNC_CMD
        do
          printf "Please start benchmark on device with IP $IP_ADDR\n" && sleep 2
        done
    fi

    $BENCHMARK_PATH/benchmark $TOPOLOGY_PATH/$TOPOLOGY --time $TIME --ipc $IPC --dir_name results/$TOPOLOGY_NAME &
    echo
done

wait
echo "Results stored in $THIS_DIR/results"

