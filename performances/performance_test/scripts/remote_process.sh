#!/bin/bash

# Script for running multiple topologies simultaneously on multiple platforms.
# Usage: bash remote_process.sh -h

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

    # IP address of remote platform
    -r|--remote_ip)
    IP_ADDR="$2"
    printf "IP_ADDR: $IP_ADDR\n"
    shift 2;;

    # Help
    -h|--help)
    printf "\nDescription:\nScript for running multiple topologies simultaneously on multiple platforms.\n\n"
    printf "Options:\n[-t|--topology_list]\n[-r|--remote_ip]\n[--ipc]:[on, off]\n"
    printf "[--rmw]:[rmw_fastrtps_cpp, rmw_cyclonedds_cpp, rmw_dps_cpp]\n"
    printf "[--time]:[test duration in seconds]\n\n"
    printf "Usage example:\n"
    printf "bash remote_process.sh -t <JSON FILE(s)> --ipc on --rmw rmw_fastrtps_cpp --time 5 -r <REMOTE_IP_ADDRESS>\n\n"
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

# Use selected RMW
export RMW_IMPLEMENTATION=$RMW

SYNC_CMD="printf 'Remote is now ready. Start benchmark\n' | nc -q 1 $IP_ADDR 1234"
if eval $SYNC_CMD
then
  printf "Connection with remote succesful\n"
else
  printf "Remote not ready. Waiting for device with IP $IP_ADDR\n"
  nc -lp 1234
fi

$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH/benchmark $TOPOLOGY_LIST --time $TIME --ipc $IPC
