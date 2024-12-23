#!/bin/bash

# Display help/usage instructions
print_help() {
  echo "Usage: $0 [OPTIONS] [TEST_DURATION]"
  echo
  echo "This script automates running benchmarks for single-process, multi-process,"
  echo "and mixed-process configurations, organizes the results, and processes them for analysis."
  echo
  echo "Options:"
  echo "  --help, -h        Display this help message."
  echo
  echo "Arguments:"
  echo "  TEST_DURATION     Time in seconds to run each benchmark. Defaults to 1 if not provided."
  echo
  echo "Description:"
  echo "  1. Benchmarks are categorized as:"
  echo "     - Single-process benchmarks [pub_sub, cli_srv, actions]."
  echo "     - Multi-process benchmarks [pub_sub, cli_srv, actions]"
  echo "     - Mix-process benchmarks [pub_sub, cli_srv, actions]"
  echo "     - Memory benchmarks [nodes, pub_sub, cli_srv]"
  echo
  echo "  2. Results are stored in a timestamped directory (e.g., results_29_11_24_18h51)."
  echo
  echo "  3. Parsing includes: (Uses python3 & dependencies)"
  echo "     - Comparing CPU, Memory & Latency (compare_metrics.py)."
  echo "     - Comparing Latency metrics (compare_latency.py)."
  echo
  echo "Example:"
  echo "  sudo env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH $0 10            Run benchmarks with TEST_DURATION=10 seconds."
  echo
  echo "Needed because sudo resets LD_LIBRARY_PATH, breaking shared library access."
  echo
  exit 0
}

# Check for help flag
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
  print_help
fi

# Check if the script is run as root
if [[ $EUID -ne 0 ]]; then
  echo "You must run this script with sudo or as root."
  echo "Usage: sudo env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH $0 [arguments]"
  exit 1
fi

# Set TEST_DURATION
TEST_DURATION=${1:-1}  # Default to 1 second if not provided

# Create the results directory
current_date=$(date +"%d_%m_%y_%Hh%M")
results_dir="results_${current_date}"
mkdir -p "$results_dir"
echo "Storing results in $results_dir"

run_benchmark() {
  local script=$1
  local config=$2

  echo "Running: $script with config: $config and test duration=$TEST_DURATION"
  TEST_DURATION="$TEST_DURATION" bash "$script" "$config"
  local exit_code=$?

  if [ $exit_code -ne 0 ]; then
    echo -e "\033[31m[ERROR] $script $config failed with exit code $exit_code\033[0m"
    exit $exit_code
  else
    echo -e "\033[32m[SUCCESS] $script $config completed successfully\033[0m"
  fi
}

# Single-process benchmarks
echo "Starting single-process benchmarks..."
cd single-process
run_benchmark "run_single_process_benchmark.sh" "pub_sub.conf"
run_benchmark "run_single_process_benchmark.sh" "cli_srv.conf"
run_benchmark "run_single_process_benchmark.sh" "actions.conf"
cd -

# Multi-process benchmarks
echo "Starting multi-process benchmarks..."
cd multi-process
run_benchmark "run_multi_process_benchmark.sh" "multi_process_pub_sub.conf"
run_benchmark "run_multi_process_benchmark.sh" "multi_process_cli_srv.conf"
run_benchmark "run_multi_process_benchmark.sh" "multi_process_actions.conf"

# Mix-process benchmarks
echo "Starting mix-process benchmarks..."
run_benchmark "run_multi_process_benchmark.sh" "mix_process_pub_sub.conf"
run_benchmark "run_multi_process_benchmark.sh" "mix_process_cli_srv.conf"
run_benchmark "run_multi_process_benchmark.sh" "mix_process_actions.conf"
cd -

# Memory benchmarks
echo "Starting memory benchmarks..."
cd memory
run_benchmark "run_memory_benchmark.sh" "memory_tests.conf" 
cd -

# Move results to the results directory
mv single-process/pub-sub* "$results_dir"
mv single-process/cli-srv* "$results_dir"
mv single-process/actions* "$results_dir"

mv multi-process/pub-sub* "$results_dir"
mv multi-process/cli-srv* "$results_dir"
mv multi-process/actions* "$results_dir"

mv memory/memory_results/ "$results_dir"

# Parse results
echo
echo "To parse results run:"
echo "  python3 parse-results/compare_metrics.py $results_dir --plot"
echo "  python3 parse-results/compare_latency.py $results_dir --plot"
echo "  python3 parse-results/compare_memory.py $results_dir"
echo
echo "Remove --plot if no plot is needed."
echo "If not already done, run 'pip3 install pandas numpy matplotlib' to install dependencies"
