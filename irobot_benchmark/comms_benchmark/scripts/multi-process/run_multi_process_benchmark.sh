#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <config_file>"
  exit 1
fi

# Load the configuration file
CONFIG_FILE=$1
if [ ! -f "$CONFIG_FILE" ]; then
  echo "Configuration file '$CONFIG_FILE' not found!"
  exit 1
fi

if [ -z "${TEST_DURATION}" ]; then
  TEST_DURATION=1
fi

# Define SCRIPT_DIR before sourcing the configuration file
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
export SCRIPT_DIR

# Source the configuration file
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
rm -rf "$OUTPUT_DIR" && mkdir -p "$OUTPUT_DIR"

# Use colors for better visualisation
GREEN="\033[32m"
RESET="\033[0m"

# Loop through RMW implementations
for RMW in "${RMW_LIST[@]}"; do
  echo -e "${GREEN}Processing RMW: $RMW${RESET}"

  # Resolve COMMS and LOANED_ENV_VARS dynamically
  declare -n COMMS="COMMS_${RMW}"
  declare -n LOANED_ENV_VARS="LOANED_ENV_VARS_${RMW}"
  echo -e "${GREEN}  COMMS for $RMW: ${COMMS[@]}${RESET}"
  echo -e "${GREEN}  LOANED_ENV_VARS for $RMW: \n      ${LOANED_ENV_VARS[@]}${RESET}"

  for COMM in "${COMMS[@]}"; do
    echo -e "${GREEN}  Testing COMM: $COMM${RESET}"

    # Set RMW implementation
    export RMW_IMPLEMENTATION="rmw_${RMW}_cpp"

    # Handle loaned environment variables
    if [[ "$COMM" == "loaned" ]]; then
      for VAR in "${LOANED_ENV_VARS[@]}"; do
        eval $VAR
      done
    fi

    # Loop through topology pairs
    for i in "${!TOPOLOGY1[@]}"; do
      T1="${TOPOLOGY1[i]}"
      T2="${TOPOLOGY2[i]}"
      RES="${RESULTS[i]}"

      RESULT_FOLDER="$OUTPUT_DIR/${RES}/${COMM}_${RMW}"
      mkdir -p "$RESULT_FOLDER"

      # Set topology files
      if [[ "$COMM" == "loaned" ]]; then
        TOP1="${TOPOLOGIES_DIR}/${T1}_loaned.json"
      else
        TOP1="${TOPOLOGIES_DIR}/${T1}.json"
      fi
      TOP2="${TOPOLOGIES_DIR}/${T2}.json"

      # Run the command
      COMMAND="${IROBOT_BENCHMARK} $TOP1 $TOP2 -x 3 --ipc off -t $TEST_DURATION -s 1000 --csv-out on"
      echo -e "${GREEN}\nCommand: \n$COMMAND\n${RESET}"
      eval $COMMAND
      if [ $? -ne 0 ]; then
        echo -e "\033[31m[ERROR] Command failed: $COMMAND\033[0m"
        exit 1
      fi

      # Move log folders
      mv *log "$RESULT_FOLDER"
    done
  done
done
