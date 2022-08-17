#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

DEFAULT_RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [[ -z "${RMW_IMPLEMENTATION}" ]]; then
  export RMW_IMPLEMENTATION=$DEFAULT_RMW_IMPLEMENTATION
fi
echo "Using RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

RMW_SPECIFIC_SETUP_SCRIPT=$THIS_DIR/$RMW_IMPLEMENTATION/setup.sh
if [ ! -f "$RMW_SPECIFIC_SETUP_SCRIPT" ]; then
    echo "$RMW_SPECIFIC_SETUP_SCRIPT does not exist."
    echo "Can't run RMW configuration for this RMW implementation."
fi

# Invoke RMW-specific script forwarding all command-line arguments to it
source $RMW_SPECIFIC_SETUP_SCRIPT "$@"
