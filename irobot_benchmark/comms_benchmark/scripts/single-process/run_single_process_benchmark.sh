#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <config_file>"
  exit 1
fi

# Define SCRIPT_DIR before sourcing the configuration file
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

# Load the configuration file
CONFIG_FILE=$1
if [ ! -f "$CONFIG_FILE" ]; then
  echo -e "\033[31m[ERROR] Configuration file '$CONFIG_FILE' not found!\033[0m"
  exit 1
fi

if [ -z "${TEST_DURATION}" ]; then
  TEST_DURATION=1
fi

# Export SCRIPT_DIR for use in the configuration file
export SCRIPT_DIR

source "$CONFIG_FILE"

# Load paths
GOVERNOR_SCRIPT="${SCRIPT_DIR}/../set_cpu_governor.sh"
IROBOT_BENCHMARK="${SCRIPT_DIR}/../../../irobot_benchmark"

# Get the original governor
original_governor=$(cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor)
# Set the CPU governor to performance
$GOVERNOR_SCRIPT performance
# Restore the original governor on exit
trap "$GOVERNOR_SCRIPT $original_governor" EXIT

if [ $? -ne 0 ]; then
  echo -e "\033[31m[ERROR] Failed to set CPU governor to performance. Exiting...\033[0m"
  exit 1
fi

# Create a directory to store log folders
SP="${PWD}/${OUTPUT_DIR}"
rm -rf "$SP" && mkdir -p "$SP"

for RMW in "${RMW_LIST[@]}"; do
  echo -e "\033[32mProcessing RMW: $RMW\033[0m"

  # Set RMW implementation
  export RMW_IMPLEMENTATION="rmw_${RMW}_cpp"

  # Resolve COMMS and LOANED_ENV_VARS dynamically
  declare -n COMMS="COMMS_${RMW}"
  declare -n LOANED_ENV_VARS="LOANED_ENV_VARS_${RMW}"
  echo -e "\033[32m  COMMS for $RMW: ${COMMS[@]}\033[0m"
  echo -e "\033[32m  LOANED_ENV_VARS for $RMW: \n     ${LOANED_ENV_VARS[@]}\033[0m"

  for COMM in "${COMMS[@]}"; do
    echo -e "\033[32m  Testing COMM: $COMM\033[0m"

    # Handle loaned environment variables
    if [[ "$COMM" == "loaned" ]]; then
      for VAR in "${LOANED_ENV_VARS[@]}"; do
        eval $VAR
      done
    fi

    IPC_OPTION="--ipc off"
    if [[ "$COMM" == "ipc_on" ]]; then
      IPC_OPTION="--ipc on"
    fi

    for TOPOLOGY in "${TOPOLOGIES[@]}"; do
      mkdir -p "$SP/${TOPOLOGY}/${RMW}_${COMM}"

      # Set topology file dynamically from configuration
      if [[ "$COMM" == "loaned" ]]; then
        TOP="${TOPOLOGIES_DIR}/${TOPOLOGY}_loaned.json"
      else
        TOP="${TOPOLOGIES_DIR}/${TOPOLOGY}.json"
      fi

      # Results folder
      RESULT_FOLDER="${RMW}_${COMM}"

      # Run the command
      COMMAND="${IROBOT_BENCHMARK} $TOP -x 3 $IPC_OPTION -t $TEST_DURATION -s 1000 --csv-out on --results-dir $RESULT_FOLDER"
      echo -e "\033[32m\nCommand: \n$COMMAND\n\033[0m"
      eval $COMMAND
      if [ $? -ne 0 ]; then
        echo -e "\033[31m[ERROR] Command failed: $COMMAND\033[0m"
        exit 1
      fi

      mv "$RESULT_FOLDER" "$SP/${TOPOLOGY}/"
    done

    # Unset loaned environment variables if applicable
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    unset RMW_FASTRTPS_USE_QOS_FROM_XML
    unset CYCLONEDDS_URI
  done
done
