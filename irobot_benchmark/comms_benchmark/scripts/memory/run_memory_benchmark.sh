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
MEMORY_TEST_DIR="../../../../memory_test"

# Create a directory to store log folders
SP="${PWD}/${OUTPUT_DIR}"
rm -rf "$SP" && mkdir -p "$SP"

# Run the memory tests
for BINARY in "${MEMORY_TEST_BINARIES[@]}"; do
  echo -e "\033[32mRunning memory test: $BINARY\033[0m"

  # Run the command
  COMMAND="${MEMORY_TEST_DIR}/${BINARY} ${SP}"
  echo -e "\033[32m\nCommand: \n$COMMAND\n\033[0m"
  eval $COMMAND
  if [ $? -ne 0 ]; then
    echo -e "\033[31m[ERROR] Command failed: $COMMAND\033[0m"
    exit 1
  fi
done