#!/bin/bash

## Test for various publishers/subscribers systems
## The ROS2 systems to be tested can be defined through environment variables


THIS_PID=$$
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
UTILITIES_DIR=$THIS_DIR/utility_scripts

# trap ctrl + c to kill all subprocesses
source $UTILITIES_DIR/kill_all_subprocesses.sh

## Export variables defined from now on
set -o allexport

## Check if variables have been defined outside this script or use default values

MAX_SERVICES=${MAX_SERVICES:=1}
MAX_CLIENTS=${MAX_CLIENTS:=5}
REQUEST_FREQUENCIES=${REQUEST_FREQUENCIES:="100"}
DURATION=${DURATION:=10}
NUM_EXPERIMENTS=${NUM_EXPERIMENTS:=1}
DIR_PATH=${DIR_PATH:=client_service_ros2}
MON_CPU_RAM=${MON_CPU_RAM:=true}


if [ -z "$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH" ]; then
  echo 'Error: env.sh has not been sourced!' >&2
  exit 1
fi

# move relative paths into results directory
if [[ $DIR_PATH != /* ]]; then
  DIR_PATH=$THIS_DIR/../results/$DIR_PATH
fi

echo "MAX_SERVICES set to $MAX_SERVICES"
echo "MAX_CLIENTS set to $MAX_CLIENTS"
echo "REQUEST_FREQUENCIES set to $REQUEST_FREQUENCIES Hz"
echo "DURATION set to $DURATION sec"
echo "NUM_EXPERIMENTS set to $NUM_EXPERIMENTS"
echo "DIR_PATH set to $DIR_PATH"
echo "MON_CPU_RAM set to $MON_CPU_RAM"


NODE="simple_client_service_main"
CMD="$ROS2_PERFORMANCE_TEST_EXECUTABLES_PATH/$NODE"

bash $UTILITIES_DIR/create_output_dir.sh $DIR_PATH

for REQUEST_FREQ in $REQUEST_FREQUENCIES;
do
  for NUM_SERVICES in `seq 1 $MAX_SERVICES`;
  do
    for NUM_CLIENTS in `seq 1 $MAX_CLIENTS`;
    do
      for ITERATION in `seq 1 $NUM_EXPERIMENTS`;
      do

        EXPERIMENT_DESCRIPTOR="$NUM_SERVICES"s_"$NUM_CLIENTS"c_"$REQUEST_FREQUENCIES"hz_"$ITERATION"

        LAT_REL_FILENAME=lat_rel_"$EXPERIMENT_DESCRIPTOR".csv
        LAT_REL_FILE_PATH=$DIR_PATH/$LAT_REL_FILENAME

        CMD_ARGS="--clients $NUM_CLIENTS --services $NUM_SERVICES --frequency $REQUEST_FREQ --duration $DURATION --filename $LAT_REL_FILE_PATH"

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

