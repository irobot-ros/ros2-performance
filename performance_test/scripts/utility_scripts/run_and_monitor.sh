#!/bin/bash
# Get CPU and memory usage of a ROS node process.
# Ouput: CPU (average, instantaneous) - RAM (used%, free [Mb])
# Note: The RAM usage is obtained from the 'free' command i.e. is not the
# 'process' memory usage, but the whole system. To clarify the influence of
# our process in the RAM usage, we start the RAM measurement a little before
# and stop after the ROS2 node process has finished.


## Check script arguments
if [ "$#" -lt 2 ]; then
    echo "Usage: ./run_and_monitor.sh <OUTPUT_FILE> <COMMAND> <OPTIONAL_COMMAND_ARGUMENTS>"
    exit 1
fi

OUTPUT_FILE_NAME=$1
shift
CMD=$1
shift
CMD_ARGS=$@

if [ ! -x "$(command -v $CMD)" ]; then
    echo "The command provided to run_and_monitor.sh does not exists!!"
    echo "$CMD"
    exit 1
fi

# TODO: this is ugly, these variables should be passed in another way
# Get some variables from parent script
msg_t=${MSG_TYPE:=NULL}
freq=${PUB_FREQ:=0}
pubs=${NUM_PUBLISHERS:=0}
subs=${NUM_SUBSCRIBERS:=0}

# Get architecture to define how to get CPU% from TOP
ARCH=$(cat /proc/cpuinfo | grep -m1 name | cut -d ' ' -f3)

## Write CSV header
echo -e "time\tcpu\trss\tvsz\tmsg_type\tpub_freq\tpubs\tsubs" > $OUTPUT_FILE_NAME
## Write NULL line for better visualization
echo -e "0\t0.0\t0\t0\t$msg_t\t$freq\t$pubs\t$subs" >> $OUTPUT_FILE_NAME

## Run the command in background process
$CMD $CMD_ARGS &
CMD_PID=$!

## Top command gets truncated by the width of the screen so we use part of it:
TOP_CMD=$(echo $CMD_PID | cut -b 1-10)

## While loop until this node is running
while  kill -0 $CMD_PID >/dev/null 2>&1
do
    if [ "$ARCH" == 'ARMv7' ]; then
        cpu="$(top -d1 -n1 | grep $TOP_CMD | cut -b 42-45 | xargs)"
    elif [ "$ARCH" == 'Intel(R)' ]; then
        cpu="$(top -n1 -b -d1 | grep $TOP_CMD | cut -b 47-53 | xargs)"
    fi

    rss_mem="$(ps --pid $CMD_PID -o rss --no-headers)"
    vsz_mem="$(ps --pid $CMD_PID -o vsz --no-headers)"

    ps_time=$(ps --pid $CMD_PID -o %t --no-headers)

    # the process already ended
    if [ -z "$ps_time" ]; then
        break
    fi

    # set internal field separator to split ps_time into a single number representing the seconds
    IFS=: read -r m s <<<"$ps_time"
    ps_time_s=$((${m#0} * 60 + ${s#0} + 1))

    if [ "$ps_time_s" != "$last_ps_time" ]
    then
       echo -e "$ps_time_s\t$cpu\t$rss_mem\t$vsz_mem\t$msg_t\t$freq\t$pubs\t$subs" >> $OUTPUT_FILE_NAME
       last_ps_time=$ps_time_s
    fi

    if [ $ps_time_s -ge 3599 ]; then
        echo "Experiment timeout reached 1 HOUR!"
        exit 1
    fi

    sleep 0.5
done
