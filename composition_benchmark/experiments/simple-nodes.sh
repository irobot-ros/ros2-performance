#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SCRIPTS_DIR="${THIS_DIR}/../scripts"
PSUTIL_SCRIPT="${SCRIPTS_DIR}/run_psutil.py"
PERF_TEST_UTILITIES_DIR="${THIS_DIR}/../../performance_test/scripts/utility_scripts"
WS_INSTALL_DIR="/root/ws/install"
TEST_BASE_DIR="${THIS_DIR}/simple-nodes"

# trap ctrl + c to kill all subprocesses
source ${PERF_TEST_UTILITIES_DIR}/kill_all_subprocesses.sh

BASE_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/base_simple_main"
COMPOSABLE_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/composable_simple_main"
COMPONENT_CONTAINER_SCRIPT="/opt/ros/rolling/lib/rclcpp_components/component_container"

do_test_manual_composition() {

  TEST_DIR="${TEST_BASE_DIR}/manual-composition"
  bash ${PERF_TEST_UTILITIES_DIR}/create_output_dir.sh ${TEST_DIR}

  for NUM_NODES in `seq 1 ${MAX_NODES}`;
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do
      TEST_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_FILE} -p ${COMPOSABLE_SCRIPT}"
      for NODES_IT in `seq 1 $NUM_NODES`;
      do
        CMD_ARGS="${CMD_ARGS} node_${NODES_IT}"
      done

      bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &

      wait

    done
  done
}

do_test_dynamic_composition() {

  TEST_DIR="${TEST_BASE_DIR}/dynamic-composition"
  bash ${PERF_TEST_UTILITIES_DIR}/create_output_dir.sh ${TEST_DIR}

  for NUM_NODES in `seq 1 ${MAX_NODES}`;
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do

      TEST_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_FILE} -p ${COMPONENT_CONTAINER_SCRIPT}"

      bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &

      for NODES_IT in `seq 1 $NUM_NODES`;
      do
        ros2 component load /ComponentManager composition_benchmark ComposableNode --no-daemon --node-name node_${NODES_IT}
      done

      wait

    done
  done
}

do_test_base_single_process() {

  TEST_DIR="${TEST_BASE_DIR}/base-single-proc"
  bash ${PERF_TEST_UTILITIES_DIR}/create_output_dir.sh ${TEST_DIR}

  for NUM_NODES in `seq 1 ${MAX_NODES}`;
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do
      TEST_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_FILE} -p ${BASE_SCRIPT}"
      for NODES_IT in `seq 1 $NUM_NODES`;
      do
        CMD_ARGS="${CMD_ARGS} node_${NODES_IT}"
      done

      bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &

      wait

    done
  done
}

do_test_base_multi_process() {
  TEST_DIR="${TEST_BASE_DIR}/base-multi-proc"
  bash ${PERF_TEST_UTILITIES_DIR}/create_output_dir.sh ${TEST_DIR}

  for NUM_NODES in `seq 1 ${MAX_NODES}`;
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do
      TEST_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_FILE}"
      for NODES_IT in `seq 1 $NUM_NODES`;
      do
        CMD_ARGS="${CMD_ARGS} -p ${BASE_SCRIPT} node_${NODES_IT}"
      done

      bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &

      wait

    done
  done
}

MAX_NODES=5
DURATION=2
NUM_EXPERIMENTS=1
do_test_manual_composition

MAX_NODES=1
DURATION=3
NUM_EXPERIMENTS=1
do_test_base_single_process
