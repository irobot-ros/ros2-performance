#!/bin/bash

run() {

    ## Check script arguments
    if [ "$#" -eq 0 ]; then
        echo "Usage: run <CMD> <CMD_ARGUMENTS>"
        exit 1
    fi

    CMD=$1
    shift
    CMD_ARGS=$@

    if [ ! -x "$(command -v $CMD)" ]; then
        echo "The command provided to run_and_monitor.sh does not exists!!"
        echo "$CMD"
        exit 1
    fi

    $CMD $CMD_ARGS &
    echo ""
}


run_and_monitor() {
  `dirname "$0"`/utility_scripts/run_and_monitor.sh "$@" &
}


create_output_dir(){
    `dirname "$0"`/utility_scripts/create_output_dir.sh "$@"
}
