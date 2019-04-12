#!/bin/bash

## Check script arguments
if [ "$#" -eq 0 ]; then
    echo "Usage: run <CMD> <OPTIONAL_COMMAND_ARGUMENTS>"
    exit 1
fi

CMD=$1
shift
CMD_ARGS=$@

if [ ! -x "$(command -v $CMD)" ]; then
    echo "The command provided to run.sh does not exists!!"
    echo "$CMD"
    exit 1
fi

$CMD $CMD_ARGS
echo ""
