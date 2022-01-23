#!/bin/bash

## Test for various publishers/subscribers systems
## The ROS2 systems to be tested can be defined through environment variables


THIS_PID=$$
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
UTILITIES_DIR=$THIS_DIR/utility_scripts

# trap ctrl + c to kill all subprocesses
source $UTILITIES_DIR/kill_all_subprocesses.sh

## Check if variables have been defined outside this script or use default values

MAX_PUBLISHERS=${MAX_PUBLISHERS:=1}
MAX_SUBSCRIBERS=${MAX_SUBSCRIBERS:=5}
MSG_TYPES=${MSG_TYPES:="10b"}
MSG_SIZE=${MSG_SIZE:=0}
PUBLISH_FREQUENCIES=${PUBLISH_FREQUENCIES:="100"}
DURATION=${DURATION:=10}
NUM_EXPERIMENTS=${NUM_EXPERIMENTS:=1}
DIR_PATH=${DIR_PATH:=pub_sub_separate_ros2}
MON_CPU_RAM=${MON_CPU_RAM:=1}



if [ -z "$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH" ]; then
  echo 'Error: env.sh has not been sourced!' >&2
  exit 1
fi

# move relative paths into results directory
if [[ $DIR_PATH != /* ]]; then
  DIR_PATH=$THIS_DIR/../results/$DIR_PATH
fi

echo "MAX_PUBLISHERS set to $MAX_PUBLISHERS"
echo "MAX_SUBSCRIBERS set to $MAX_SUBSCRIBERS"
echo "MSG_TYPES set to $MSG_TYPES"
echo "MSG_SIZE set to $MSG_SIZE"
echo "PUBLISH_FREQUENCIES set to $PUBLISH_FREQUENCIES Hz"
echo "DURATION set to $DURATION sec"
echo "NUM_EXPERIMENTS set to $NUM_EXPERIMENTS"
echo "DIR_PATH set to $DIR_PATH"
echo "MON_CPU_RAM set to $MON_CPU_RAM"

# in this multiprocess experiment we make some assumptions on the maximum time that nodes may take in order to be created.
# if there are many nodes on low performing platforms, these assumptions may be incorrect.
# NOTE: the experiments will work in any case, but if the assumptions are not respected the reliability measure will be wrong
# this is the workflow
# start creating subscribers in background
# after 2 seconds start creating publisher
# subscribers will spin for 5 seconds more than experiment duration
# publisher will start publishing as soon as they are created
# with these default values, the assumptions are:
# 1) subscribers take less than (2 + publisher creation time) seconds to be created
# 2) publishers take less than (5 - 2) seconds to be created

NODES_WARNING_LIMIT=10
SUBS_WARNING_LIMIT=$(($NODES_WARNING_LIMIT + $MAX_PUBLISHERS))
if [ "$MAX_PUBLISHERS" -gt "$NODES_WARNING_LIMIT" ]; then
  echo "WARNING: due to the high number of publishers, the reliability measure may be inaccurate"
fi

if [ "$MAX_SUBSCRIBERS" -gt "$SUBS_WARNING_LIMIT" ]; then
  echo "WARNING: due to the high number of subscribers, the reliability measure may be inaccurate"
fi

PUB_NODE="publisher_nodes_main"
SUB_NODE="subscriber_nodes_main"
PUB_CMD="$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH/$PUB_NODE"
SUB_CMD="$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH/$SUB_NODE"

bash $UTILITIES_DIR/create_output_dir.sh $DIR_PATH

for MSG_TYPE in $MSG_TYPES
do
  for PUB_FREQ in $PUBLISH_FREQUENCIES;
  do
    for NUM_PUBLISHERS in `seq 1 $MAX_PUBLISHERS`;
    do
      for NUM_SUBSCRIBERS in `seq 1 $MAX_SUBSCRIBERS`;
      do
        for ITERATION in `seq 1 $NUM_EXPERIMENTS`;
        do

          MSG_DESCRIPTOR=$MSG_TYPE
          if [ $MSG_TYPE = "vector" ]; then
            MSG_DESCRIPTOR="$MSG_DESCRIPTOR"_"$MSG_SIZE"
          fi
          EXPERIMENT_DESCRIPTOR="$NUM_PUBLISHERS"p_"$NUM_SUBSCRIBERS"s_"$MSG_DESCRIPTOR"_"$PUB_FREQ"hz_"$ITERATION"

          CMD_ARGS="--subs $NUM_SUBSCRIBERS --pubs $NUM_PUBLISHERS
          --msg_type $MSG_TYPE
          --duration $DURATION
          --experiment_path $DIR_PATH --monitor_stats $MON_CPU_RAM"

          SUB_CMD_ARGS=$CMD_ARGS"
          --experiment_name sub_$EXPERIMENT_DESCRIPTOR"

          PUB_CMD_ARGS=$CMD_ARGS"
          --msg_size $MSG_SIZE --frequency $PUB_FREQ
          --experiment_name pub_$EXPERIMENT_DESCRIPTOR"

          bash $UTILITIES_DIR/run.sh $SUB_CMD $SUB_CMD_ARGS &
          sleep 2
          bash $UTILITIES_DIR/run.sh $PUB_CMD $PUB_CMD_ARGS &

          wait

       done
     done
   done
  done
done
