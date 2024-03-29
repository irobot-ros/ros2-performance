#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

SCRIPTS_DIR="${THIS_DIR}/../../scripts"
PSUTIL_SCRIPT="${SCRIPTS_DIR}/performance/run_psutil.py"
UTILITIES_SCRIPT_DIR="${SCRIPTS_DIR}/utilities"
WS_INSTALL_DIR="/root/ws/install"
TEST_BASE_DIR="${THIS_DIR}/../../../_results/composition/simple-nodes"

# trap ctrl + c to kill all subprocesses
source ${UTILITIES_SCRIPT_DIR}/kill_all_subprocesses.sh

BASE_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/base_simple_main"
COMPOSABLE_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/composable_simple_main"
COMPONENT_CONTAINER_SCRIPT="/opt/ros/humble/lib/rclcpp_components/component_container"

do_test_manual_composition() {
  TEST_DIR="${TEST_BASE_DIR}/manual-composition"
  bash ${UTILITIES_SCRIPT_DIR}/create_output_dir.sh ${TEST_DIR}
  TEST_SUITE_STD_LOG_FILE="${TEST_DIR}/std_log.txt"

  for NUM_NODES in ${ALL_NUM_NODES};
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do
      TEST_LOG_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_LOG_FILE} -p ${COMPOSABLE_SCRIPT}"
      for NODE_IT in `seq 1 $NUM_NODES`;
      do
        CMD_ARGS="${CMD_ARGS} node_${NODE_IT}"
      done

      echo "do_test_manual_composition: ${PSUTIL_SCRIPT} ${CMD_ARGS}"
      bash ${UTILITIES_SCRIPT_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &>> ${TEST_SUITE_STD_LOG_FILE} &

      wait
    done
  done
}

do_test_dynamic_composition() {
  TEST_DIR="${TEST_BASE_DIR}/dynamic-composition"
  bash ${UTILITIES_SCRIPT_DIR}/create_output_dir.sh ${TEST_DIR}
  TEST_SUITE_STD_LOG_FILE=${TEST_DIR}/std_log.txt

  for NUM_NODES in ${ALL_NUM_NODES};
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do
      TEST_LOG_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_LOG_FILE} -p ${COMPONENT_CONTAINER_SCRIPT}"

      echo "do_test_dynamic_composition: ${PSUTIL_SCRIPT} ${CMD_ARGS}"
      bash ${UTILITIES_SCRIPT_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &>> ${TEST_SUITE_STD_LOG_FILE} &

      for NODE_IT in `seq 1 $NUM_NODES`;
      do
        ros2 component load /ComponentManager composition_benchmark ComposableNode --no-daemon --node-name node_${NODE_IT}
      done

      wait
    done
  done
}

do_test_base_single_process() {
  TEST_DIR="${TEST_BASE_DIR}/base-single-proc"
  bash ${UTILITIES_SCRIPT_DIR}/create_output_dir.sh ${TEST_DIR}
  TEST_SUITE_STD_LOG_FILE=${TEST_DIR}/std_log.txt

  for NUM_NODES in ${ALL_NUM_NODES};
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do
      TEST_LOG_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_LOG_FILE} -p ${BASE_SCRIPT}"
      for NODE_IT in `seq 1 $NUM_NODES`;
      do
        CMD_ARGS="${CMD_ARGS} node_${NODE_IT}"
      done

      echo "do_test_base_single_process: ${PSUTIL_SCRIPT} ${CMD_ARGS}"
      bash ${UTILITIES_SCRIPT_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &>> ${TEST_SUITE_STD_LOG_FILE} &

      wait
    done
  done
}

do_test_base_multi_process() {
  TEST_DIR="${TEST_BASE_DIR}/base-multi-proc"
  bash ${UTILITIES_SCRIPT_DIR}/create_output_dir.sh ${TEST_DIR}
  TEST_SUITE_STD_LOG_FILE=${TEST_DIR}/std_log.txt

  for NUM_NODES in ${ALL_NUM_NODES};
  do
    for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
    do
      TEST_LOG_FILE="${TEST_DIR}/${NUM_NODES}_nodes_${EXP_IT}_it.txt"

      CMD_ARGS="-t ${DURATION} --file ${TEST_LOG_FILE}"
      for NODE_IT in `seq 1 $NUM_NODES`;
      do
        CMD_ARGS="${CMD_ARGS} -p ${BASE_SCRIPT} node_${NODE_IT}"
      done

      echo "do_test_base_multi_process: ${PSUTIL_SCRIPT} ${CMD_ARGS}"
      bash ${UTILITIES_SCRIPT_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &>> ${TEST_SUITE_STD_LOG_FILE} &

      wait
    done
  done
}

NUM_EXPERIMENTS=5
DURATION=40
ALL_NUM_NODES="1 2 3 5 10 15 20"

do_test_base_single_process
do_test_base_multi_process
do_test_manual_composition
do_test_dynamic_composition
