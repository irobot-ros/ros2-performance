#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SCRIPTS_DIR="${THIS_DIR}/../scripts"
PSUTIL_SCRIPT="${SCRIPTS_DIR}/run_psutil.py"
PERF_TEST_UTILITIES_DIR="${THIS_DIR}/../../performance_test/scripts/utility_scripts"
WS_INSTALL_DIR="/root/ws/install"
TEST_BASE_DIR="${THIS_DIR}/simple-nodes"

# trap ctrl + c to kill all subprocesses
#source ${PERF_TEST_UTILITIES_DIR}/kill_all_subprocesses.sh

PUBLISHER_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/publisher_main"
SUBSCRIBER_SCRIPT="${WS_INSTALL_DIR}/composition_benchmark/lib/composition_benchmark/subscriber_main"
COMPONENT_CONTAINER_SCRIPT="/opt/ros/rolling/lib/rclcpp_components/component_container"

#CMD_ARGS="-t 20 -p ${SUBSCRIBER_SCRIPT} -p ${PUBLISHER_SCRIPT}"
#bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &
#wait

CMD_ARGS="-t 6 -p ${COMPONENT_CONTAINER_SCRIPT}"
bash ${PERF_TEST_UTILITIES_DIR}/run.sh python3 ${PSUTIL_SCRIPT} ${CMD_ARGS} &
ros2 component load /ComponentManager composition_benchmark ComposablePublisher --no-daemon --node-name pub_node -p "topic:=test_topic" -p frequency:=500 -p size:=20
ros2 component load /ComponentManager composition_benchmark ComposableSubscriber --no-daemon --node-name sub_node -p "topic:=test_topic"
wait
