#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SCRIPTS_DIR="${THIS_DIR}/../scripts"
PSUTIL_SCRIPT="${SCRIPTS_DIR}/run_psutil.py"
PERF_TEST_UTILITIES_DIR="${THIS_DIR}/../../performance_test/scripts/utility_scripts"
WS_INSTALL_DIR="/root/ws/install"
TEST_BASE_DIR="${THIS_DIR}/simple-nodes"

# trap ctrl + c to kill all subprocesses
source ${PERF_TEST_UTILITIES_DIR}/kill_all_subprocesses.sh

PUBLISHER_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/base_publisher_main"
SUBSCRIBER_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/base_subscriber_main"
MANUAL_COMPOSITION_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/composable_pub_sub_main"
COMPONENT_CONTAINER_SCRIPT="/opt/ros/rolling/lib/rclcpp_components/component_container"

do_test_multi_process() {
  CMD_ARGS="-t 20 -p ${SUBSCRIBER_SCRIPT} -p ${PUBLISHER_SCRIPT}"
  bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &
  wait
}

do_test_dynamic_composition() {
  CMD_ARGS="-t 6 -p ${COMPONENT_CONTAINER_SCRIPT}"
  bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &
  ros2 component load /ComponentManager composition_benchmark ComposablePublisher --no-daemon --node-name pub_node -p "topic:=test_topic" -p frequency:=500 -p size:=20
  ros2 component load /ComponentManager composition_benchmark ComposableSubscriber --no-daemon --node-name sub_node -p "topic:=test_topic"
  wait
}

do_test_manual_composition() {
  DURATION=15
  NUM_SUBS=2
  PUB_FREQUENCY=10
  MSG_SIZE=10
  USE_IPC=1
  SPIN_TYPE="spin_future"
  CMD_ARGS="-t ${DURATION} -p ${MANUAL_COMPOSITION_SCRIPT} ${NUM_SUBS} ${PUB_FREQUENCY} ${MSG_SIZE} ${USE_IPC} ${SPIN_TYPE}"
  bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &
  wait
}

do_test_manual_composition
