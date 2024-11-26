#!/bin/bash

# Function to display help
print_help() {
  echo "Usage: $0 <governor>"
  echo
  echo "This script sets the CPU governor to the specified value for all policies."
  echo
  echo "Examples:"
  echo "  $0 performance       # Set the CPU governor to 'performance'."
  echo "  $0 powersave         # Set the CPU governor to 'powersave'."
  echo
  exit 0
}

# Check if arguments are provided
if [[ $# -ne 1 || "$1" == "--help" || "$1" == "-h" ]]; then
  print_help
fi

# Get the desired governor from the argument
desired_governor="$1"

# Ensure the CPU frequency scaling directory exists
if [ ! -d "/sys/devices/system/cpu/cpufreq/" ]; then
  echo -e "\033[31m[ERROR] CPU frequency scaling not supported on this system.\033[0m"
  exit 1
fi

# Apply the specified governor to all policies
echo "Setting CPU governor to '$desired_governor'..."
for policy in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
  if ! sudo echo "$desired_governor" > "$policy" 2>/dev/null; then
    echo -e "\033[31m[ERROR] Permission denied while setting governor for $policy - run with sudo.\033[0m"
    exit 1
  fi
done

# Verify the change
current_governor=$(cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor)
if [ "$current_governor" != "$desired_governor" ]; then
  echo -e "\033[31m[ERROR] Failed to set CPU governor to '$desired_governor'. Current value: '$current_governor'.\033[0m"
  exit 1
fi

echo -e "\033[32m[SUCCESS] CPU governor set to '$desired_governor'.\033[0m"
