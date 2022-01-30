#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SCRIPTS_DIR="${THIS_DIR}/../scripts"
PSUTIL_SCRIPT="${SCRIPTS_DIR}/run_psutil.py"
PERF_TEST_UTILITIES_DIR="${THIS_DIR}/../../performance_test/scripts/utility_scripts"
WS_INSTALL_DIR="/root/ws/install"
TEST_BASE_DIR="${THIS_DIR}/pub-sub"

# trap ctrl + c to kill all subprocesses
source ${PERF_TEST_UTILITIES_DIR}/kill_all_subprocesses.sh

PUBLISHER_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/base_publisher_main"
SUBSCRIBER_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/base_subscriber_main"
MANUAL_COMPOSITION_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/composable_pub_sub_main"
COMPONENT_CONTAINER_SCRIPT="/opt/ros/rolling/lib/rclcpp_components/component_container"

do_test_multi_process() {
  TEST_DIR="${TEST_BASE_DIR}/multi-process"
  bash ${PERF_TEST_UTILITIES_DIR}/create_output_dir.sh ${TEST_DIR}
  TEST_SUITE_STD_LOG_FILE=${TEST_DIR}/std_log.txt

  for NUM_SUBS in ${ALL_NUM_SUBS};
  do
    for PUB_FREQ in ${ALL_PUB_FREQUENCIES};
    do
      for MSG_SIZE in ${ALL_MSG_SIZES};
      do
        for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
        do
          TEST_LOG_FILE="${TEST_DIR}/${NUM_SUBS}_subs_${PUB_FREQ}_freq_${MSG_SIZE}_size_${EXP_IT}_it.txt"
          CMD_ARGS="-t ${DURATION} --file ${TEST_LOG_FILE}"

          for SUB_IT in `seq 1 $NUM_SUBS`;
          do
            CMD_ARGS="${CMD_ARGS} -p ${SUBSCRIBER_SCRIPT} name sub_${SUB_IT}"
          done
          CMD_ARGS="${CMD_ARGS} -p ${PUBLISHER_SCRIPT} name pub freq ${PUB_FREQ} size ${MSG_SIZE} ipc 0"

          echo "do_test_multi_process: ${PSUTIL_SCRIPT} ${CMD_ARGS}"
          bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &>> ${TEST_SUITE_STD_LOG_FILE} &

          wait
        done
      done
    done
  done
}

do_test_dynamic_composition() {
  TEST_DIR="${TEST_BASE_DIR}/dynamic-composition"
  bash ${PERF_TEST_UTILITIES_DIR}/create_output_dir.sh ${TEST_DIR}
  TEST_SUITE_STD_LOG_FILE=${TEST_DIR}/std_log.txt

  for NUM_SUBS in ${ALL_NUM_SUBS};
  do
    for PUB_FREQ in ${ALL_PUB_FREQUENCIES};
    do
      for MSG_SIZE in ${ALL_MSG_SIZES};
      do
        for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
        do
          TEST_LOG_FILE="${TEST_DIR}/${NUM_SUBS}_subs_${PUB_FREQ}_freq_${MSG_SIZE}_size_${EXP_IT}_it.txt"
          CMD_ARGS="-t ${DURATION} --file ${TEST_LOG_FILE} -p ${COMPONENT_CONTAINER_SCRIPT}"

          echo "do_test_dynamic_composition: ${PSUTIL_SCRIPT} ${CMD_ARGS}"
          bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &>> ${TEST_SUITE_STD_LOG_FILE} &

          for SUB_IT in `seq 1 $NUM_SUBS`;
          do
            ros2 component load /ComponentManager composition_benchmark ComposableSubscriber --no-daemon --node-name sub_${SUB_IT}
          done
          ros2 component load /ComponentManager composition_benchmark ComposablePublisher --no-daemon --node-name pub_node -p frequency:=${PUB_FREQ} -p size:=${MSG_SIZE}
          wait
        done
      done
    done
  done
}

do_test_manual_composition() {
  TEST_DIR="${TEST_BASE_DIR}/manual-composition"
  bash ${PERF_TEST_UTILITIES_DIR}/create_output_dir.sh ${TEST_DIR}
  TEST_SUITE_STD_LOG_FILE=${TEST_DIR}/std_log.txt

  for NUM_SUBS in ${ALL_NUM_SUBS};
  do
    for PUB_FREQ in ${ALL_PUB_FREQUENCIES};
    do
      for MSG_SIZE in ${ALL_MSG_SIZES};
      do
        for EXP_IT in `seq 1 ${NUM_EXPERIMENTS}`;
        do
          TEST_LOG_FILE="${TEST_DIR}/${NUM_SUBS}_subs_${PUB_FREQ}_freq_${MSG_SIZE}_size_${EXP_IT}_it.txt"

          SCRIPT_ARGS="subs ${NUM_SUBS} freq ${PUB_FREQ} size ${MSG_SIZE} ipc 0 spin_t spin"
          CMD_ARGS="-t ${DURATION} --file ${TEST_LOG_FILE} -p ${MANUAL_COMPOSITION_SCRIPT} ${SCRIPT_ARGS}"

          echo "do_test_manual_composition: ${PSUTIL_SCRIPT} ${CMD_ARGS}"
          bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &>> ${TEST_SUITE_STD_LOG_FILE} &
          wait
        done
      done
    done
  done
}

ALL_NUM_SUBS="1 2 5"
ALL_PUB_FREQUENCIES="10 50 100"
ALL_MSG_SIZES="1 50000 100000"
NUM_EXPERIMENTS=1
DURATION=10

#ALL_IPC_VALS="0 1"
#ALL_SPIN_TYPES="spin"

#do_test_manual_composition
#do_test_dynamic_composition
do_test_multi_process
