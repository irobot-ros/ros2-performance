#!/bin/bash

# Display help/usage instructions
print_help() {
  echo "Usage: $0 [FOLDER]"
  echo
  echo "This script analyzes benchmark requirements using CSV files from the specified folder."
  echo "It is work in progress - not finished yet."
  echo
  echo "Arguments:"
  echo "  FOLDER            Path to the folder containing 'average_latency.csv' and 'average_metrics.csv'."
  echo
  echo "Description:"
  echo "  The script verifies multiple requirements by extracting and analyzing data from the provided CSV files."
  echo
  echo "Examples:"
  echo "  $0 ./results_folder   Analyze data from the 'results_folder'."
  echo
  exit 0
}

# Check for help flag or no arguments
if [[ "$1" == "--help" || "$1" == "-h" || -z "$1" ]]; then
  print_help
fi

# Validate the folder argument
folder="$1"
if [[ ! -d "$folder" ]]; then
  echo -e "\033[31m[ERROR] The folder '$folder' does not exist or is not accessible.\033[0m"
  exit 1
fi

# Ensure required CSV files exist in the folder
latency_csv="${folder}/average_latency.csv"
metrics_csv="${folder}/average_metrics.csv"
if [[ ! -f "$latency_csv" || ! -f "$metrics_csv" ]]; then
  echo -e "\033[31m[ERROR] Missing 'average_latency.csv' or 'average_metrics.csv' in folder '$folder'.\033[0m"
  exit 1
fi

echo "Analyzing CSV files from folder: $folder"

# ------------------------------------------------------------------------------------------------------------
echo "### 1. Latency Consistency Across Message Sizes"
echo -e "Requirement: CPU and latency should not depend on message size for single-process applications. \n"
output=$(cat "$latency_csv" | grep pub-sub | grep single | grep loaned | grep fast)
echo -e "rclcpp intra-process on: \n"
head -n 1 "$latency_csv"
echo -e "$output \n"
output=$(cat "$latency_csv" | grep pub-sub | grep single | grep ipc_off | grep fast)
echo -e "Shared Memory (Loaned messages): \n"
head -n 1 "$latency_csv"
echo -e "$output \n"
output=$(cat "$latency_csv" | grep pub-sub | grep single | grep ipc_on | grep fast)
# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 2. Latency and CPU Comparison: Multi-Process vs Single-Process \n"
echo -e "Requirement: Multi-process latency/CPU should be ≈ single-process for POD message types. \n"
echo -e "Multi-Process (Shared Memory): \n"
latency_output=$(cat "$latency_csv" | grep pub-sub | grep multi | grep loaned | grep fast)
echo -e "Latency: \n"
head -n 1 "$latency_csv"
echo -e "$latency_output \n"
metrics_output=$(cat "$metrics_csv" | grep pub-sub | grep multi | grep loaned | grep fast)
echo -e "Metrics: \n"
head -n 1 "$metrics_csv"
echo -e "$metrics_output \n"
echo -e "Single-Process (Shared Memory): \n"
latency_output=$(cat "$latency_csv" | grep pub-sub | grep single | grep loaned | grep fast)
echo -e "Latency: \n"
head -n 1 "$latency_csv"
echo -e "$latency_output \n"
metrics_output=$(cat "$metrics_csv" | grep pub-sub | grep single | grep loaned | grep fast)
echo -e "Metrics: \n"
head -n 1 "$metrics_csv"
echo -e "$metrics_output \n"
# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 3. Long-Running Stability \n"
echo -e "Requirement: CPU, latency, and RAM must remain constant over long durations (>10 minutes). \n"
latency_output=$(cat average_latency.csv | grep white_mountain_fixed_size | grep loaned | grep fast)
echo -e "Latency: \n"
head -n 1 average_latency.csv
echo -e "$latency_output \n"
metrics_output=$(cat average_metrics.csv | grep white_mountain_fixed_size | grep loaned | grep fast)
echo -e "Metrics: \n"
head -n 1 average_metrics.csv
echo -e "$metrics_output \n"
# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 4. Multi-Transport Communication \n"
echo -e "Requirement: Combining multiple transports should not degrade performance. \n"
latency_output=$(cat average_latency.csv | grep pub-sub | grep mix_process | grep loaned | grep fast)
echo -e "Latency: \n"
head -n 1 average_latency.csv
echo -e "$latency_output \n"
metrics_output=$(cat average_metrics.csv | grep pub-sub | grep mix_process | grep loaned | grep fast)
echo -e "Metrics: \n"
head -n 1 average_metrics.csv
echo -e "$metrics_output \n"
# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 5. Scalability with Number of Clients (Services) \n"
echo -e "Requirement: Service performance depends only on active clients, not the total number. \n"
latency_output=$(cat average_latency.csv | grep cli-srv | grep multi | grep loaned | grep fast)
echo -e "Latency: \n"
head -n 1 average_latency.csv
echo -e "$latency_output \n"
metrics_output=$(cat average_metrics.csv | grep cli-srv | grep multi | grep loaned | grep fast)
echo -e "Metrics: \n"
head -n 1 average_metrics.csv
echo -e "$metrics_output \n"
# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 6. Scalability with Number of Clients (Actions) \n"
echo -e "Requirement: Action performance depends only on active clients, not the total number. \n"
latency_output=$(cat average_latency.csv | grep action | grep multi | grep loaned | grep fast)
echo -e "Latency: \n"
head -n 1 average_latency.csv
echo -e "$latency_output \n"
metrics_output=$(cat average_metrics.csv | grep action | grep multi | grep loaned | grep fast)
echo -e "Metrics: \n"
head -n 1 average_metrics.csv
echo -e "$metrics_output \n"
# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 7. CPU Overhead with Remote Subscribers \n"
echo -e "Requirement: Adding remote subscribers should add <10% CPU overhead. \n"
metrics_output_1mb=$(cat average_metrics.csv | grep pub-sub | grep mix_process | grep loaned | grep fast | grep 1mb)
echo -e "1MB Metrics: \n"
head -n 1 average_metrics.csv
echo -e "$metrics_output_1mb \n"
metrics_output_4mb=$(cat average_metrics.csv | grep pub-sub | grep mix_process | grep loaned | grep fast | grep 4mb)
echo -e "4MB Metrics: \n"
head -n 1 average_metrics.csv
echo -e "$metrics_output_4mb \n"
# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 8. Scalability of RAM Usage \n"
echo -e "Requirement: RAM usage scales linearly with processes or entities. \n"
# metrics_output_pubsub=$(cat average_metrics.csv | grep pub-sub | grep multi | grep loaned | grep fast)
# echo -e "Pub-Sub Metrics: \n"
# head -n 1 average_metrics.csv
# echo -e "$metrics_output_pubsub \n"
# metrics_output_clisrv=$(cat average_metrics.csv | grep cli-srv | grep multi | grep loaned | grep fast)
# echo -e "CLI-SRV Metrics: \n"
# head -n 1 average_metrics.csv
# echo -e "$metrics_output_clisrv \n"
# Create a temporary directory to store the results

# ------------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------------
echo -e "### 9. Lifecycle Mechanism Efficiency \n"
echo -e "Requirement: Autocore lifecycle mechanisms must have less CPU overhead than callback-based 'early return'. \n"
echo -e ".. no command for this yet .. \n"

echo -e "### 10. Flash Size Limit \n"
echo -e "Requirement: Total flash size of Autocore RMW and dependencies must be <4MB. \n"
echo -e ".. no command for this yet .. \n"

echo -e "### 11. Baseline Comparison \n"
echo -e "Requirement: Autocore RMW must show no regressions compared to Fast-DDS. \n"
echo -e "Run all relevant commands for both 'fast' and 'cyclone' entries. \n"
echo -e ".. no automated script yet .. \n"
# ------------------------------------------------------------------------------------------------------------
