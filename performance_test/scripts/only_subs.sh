#!/bin/bash

## Test for onl;y subscribers systems
## The ROS2 systems to be tested can be defined through environment variables
## It is almost the same as pub_sub_ros2.sh except that
## PUBLISH_FREQUENCIES and MSG_SIZE are not used
## Note that you can still create dynamic size subscriptions



THIS_PID=$$
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
UTILITIES_DIR=$THIS_DIR/utility_scripts

# trap ctrl + c to kill all subprocesses
source $UTILITIES_DIR/kill_all_subprocesses.sh

## Export variables defined from now on
set -o allexport

## Check if variables have been defined outside this script or use default values

MAX_PUBLISHERS=${MAX_PUBLISHERS:=1}
MAX_SUBSCRIBERS=${MAX_SUBSCRIBERS:=5}
MSG_TYPES=${MSG_TYPES:="10b"}
DURATION=${DURATION:=10}
NUM_EXPERIMENTS=${NUM_EXPERIMENTS:=1}
DIR_PATH=${DIR_PATH:=only_subs_ros2}
MON_CPU_RAM=${MON_CPU_RAM:=true}


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
echo "DURATION set to $DURATION sec"
echo "NUM_EXPERIMENTS set to $NUM_EXPERIMENTS"
echo "DIR_PATH set to $DIR_PATH"
echo "MON_CPU_RAM set to $MON_CPU_RAM"

NODE="subscriber_nodes_main"
CMD="$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH/$NODE"

bash $UTILITIES_DIR/create_output_dir.sh $DIR_PATH

for MSG_TYPE in $MSG_TYPES
do
  for NUM_PUBLISHERS in `seq 1 $MAX_PUBLISHERS`;
  do
    for NUM_SUBSCRIBERS in `seq 1 $MAX_SUBSCRIBERS`;
    do
      for ITERATION in `seq 1 $NUM_EXPERIMENTS`;
      do

        MSG_DESCRIPTOR=$MSG_TYPE

        EXPERIMENT_DESCRIPTOR="$NUM_PUBLISHERS"p_"$NUM_SUBSCRIBERS"s_"$MSG_DESCRIPTOR"_"$PUB_FREQ"hz_"$ITERATION"

        LAT_REL_FILENAME=lat_rel_"$EXPERIMENT_DESCRIPTOR".csv
        LAT_REL_FILE_PATH=$DIR_PATH/$LAT_REL_FILENAME

        CMD_ARGS="--subs $NUM_SUBSCRIBERS --pubs $NUM_PUBLISHERS --msg_type $MSG_TYPE --duration $DURATION --executors 0 --filename $LAT_REL_FILE_PATH"

        if ! $MON_CPU_RAM ; then
          # No CPU & RAM measurement
          bash $UTILITIES_DIR/run.sh $CMD $CMD_ARGS &
        else
          # I want CPU & RAM measurement
          CPU_RAM_FILENAME=cpu_ram_"$EXPERIMENT_DESCRIPTOR".csv
          CPU_RAM_FILE_PATH=$DIR_PATH/$CPU_RAM_FILENAME
          bash $UTILITIES_DIR/run_and_monitor.sh $CPU_RAM_FILE_PATH $CMD $CMD_ARGS &
        fi
        wait

      done
    done
  done
done
