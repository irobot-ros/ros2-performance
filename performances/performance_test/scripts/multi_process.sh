#!/bin/bash

# Script for running benchmark + debug_node
# Usage: bash multi_process.sh -h

# Parse input arguments
if [[ $# -eq 0 ]]; then set -- "-h"; fi

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -t|--topology_list)
    TOPOLOGY_LIST="$2"

    while [[ "${3::1}" != '-' ]]
    do
        shift
        TOPOLOGY_LIST="$TOPOLOGY_LIST $2"
    done
    shift 2;;

    # IPC mode
    --ipc)
    IPC="$2"
    printf "IPC: $IPC\n"
    shift 2;;

    --rmw)
    RMW="$2"
    printf "RMW: $RMW\n"
    shift 2;;

    # Duration of the test
    --time)
    TIME="$2"
    printf "TIME: $TIME seconds\n"
    shift 2;;

    # IP address of local platform which runs the benchmark
    --ip)
    IP_ADDR="$2"
    printf "IP_ADDR: $IP_ADDR\n"
    shift 2;;

    # Sync flag: Wait for active counterparto to start benchmark
    -s|--sync)
    SYNC=true
    printf "SYNC: true\n"
    shift;;

    # Help
    -h|--help)
    printf "\nDescription: Script for running multiple topologies simultaneously, on single or multiple platforms.\n\n"
    printf "Options:\n[-t|--topology_list]\n[--ipc]:[on, off]\n"
    printf "[--rmw]:[rmw_fastrtps_cpp, rmw_cyclonedds_cpp, rmw_dps_cpp]\n"
    printf "[--time]:[test duration in seconds]\n[-s|--sync]\n[--ip]\n\n"
    printf "Single platform usage example:\n"
    printf "bash multi_process.sh -t <JSON FILE(s)> --ipc on --rmw rmw_fastrtps_cpp --time 5\n\n"
    printf "Multi platform usage example:\n"
    printf "* Local:\n"
    printf "bash multi_process.sh -t <JSON FILE(s)> --ipc on --rmw rmw_fastrtps_cpp --time 5 -s\n\n"
    printf "* Remote: (use the IP address of the local device)\n"
    printf "bash multi_process.sh -t <JSON FILE(s)> --ipc on --rmw rmw_fastrtps_cpp --time 5 -s --ip 192.168.1.218\n\n"
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

# Use selected RMW
export RMW_IMPLEMENTATION=$RMW

for FILE in $TOPOLOGY_LIST
do
    TOPOLOGY_NAME=$(basename $FILE)
    printf "\nRunning $TOPOLOGY_NAME in a background process.\n"

    TOPOLOGY_RESULTS_DIR=${TOPOLOGY_NAME::-5}_log

    if [[ ! -z "$SYNC" && -z "$IP_ADDR" ]]
    then
        printf "\nWaiting for remote benchmark to start...\n"
        nc -l $IP_ADDR -p 1234

    elif [[ ! -z "$SYNC" ]]
    then
        SYNC_CMD="printf 'Remote device says: Start benchmark' | nc -q 1 $IP_ADDR 1234"
        until eval $SYNC_CMD
        do
          printf "Please run multi_process.sh on device with IP $IP_ADDR\n" && sleep 2
        done
    fi

    $BENCHMARK_PATH/benchmark $FILE --time $TIME --ipc $IPC --dir_name results/$TOPOLOGY_RESULTS_DIR &
    echo
done

wait
echo "Results stored in $THIS_DIR/results"

