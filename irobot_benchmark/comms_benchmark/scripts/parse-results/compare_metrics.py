#!/usr/bin/env python3

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def get_sorted_directories_by_mtime(directories):
    # Sort directories by the modification time of resources.txt
    return sorted(directories, key=lambda d: os.path.getmtime(os.path.join(d, "resources.txt")))

def find_directories_with_resources(root_dirs):
    directories = []
    for root_dir in root_dirs:
        for root, _, files in os.walk(root_dir):
            if "resources.txt" in files:
                # Remove ./ prefix from the path
                clean_path = root.lstrip("./")
                directories.append(clean_path)
    return directories

def process_directory(directory):
    file_path = os.path.join(directory, "resources.txt")

    # Read the file
    resources_data = pd.read_csv(file_path)

    # Extract CPU percentage, RSS memory, VSZ memory, and Latency
    cpu_perc = resources_data['cpu_perc'].astype(float)
    rss_KB = resources_data['rss_KB'].astype(float)
    vsz_KB = resources_data['vsz_KB'].astype(float)
    resources_data['latency_us'] = resources_data['latency_us'].apply(lambda x: 0 if x > 1000000000 else x)
    latency_us = resources_data['latency_us'].astype(float)
    time_ms = resources_data['time_ms']

    return time_ms, cpu_perc, rss_KB, vsz_KB, latency_us

def print_help():
    """
    Print usage instructions for the script.
    """
    print("""
Usage: python compare_metrics.py <directory1> [<directory2> ...] [--plot]

Arguments:
  <directory1>, <directory2>, ...   Directories to search for subdirectories containing 'resources.txt' files.
  --plot                            Optional flag to generate plots for CPU, RSS, VSZ, and Latency metrics over time.

Example:
  python3 compare_metrics.py ./dir1 ./dir2 --plot
""")
    sys.exit(0)

# Parse command-line arguments for directories and optional plot
if "--help" in sys.argv or "-h" in sys.argv:
    print_help()

if len(sys.argv) < 2:
    print("Error: Missing required arguments.")
    print("Use --help or -h for usage instructions.")
    sys.exit(1)

args = sys.argv[1:]
plot = "--plot" in args
directories = [arg for arg in args if arg != "--plot"]

if not directories:
    print("Error: No directories provided.")
    sys.exit(1)

# Find directories with resources.txt in the specified directories
directories = find_directories_with_resources(directories)

if not directories:
    print("No directories with 'resources.txt' found in the provided directories.")
    exit(1)

# Sort directories by modification time (oldest to newest)
directories = get_sorted_directories_by_mtime(directories)

# Lists to store data from all directories
all_time_ms = []
all_cpu_perc = []
all_rss_MB = []
all_vsz_MB = []
all_latency_us = []

# CSV output file
csv_output_file = "average_metrics.csv"

# Process each directory and collect data
with open(csv_output_file, 'w') as csv_file:
    csv_file.write("Directory;CPU;RSS_MB;VSZ_MB;Latency_us\n")

    for directory in directories:
        time_ms, cpu_perc, rss_KB, vsz_KB, latency_us = process_directory(directory)
        all_time_ms.append(time_ms)
        all_cpu_perc.append(cpu_perc)
        all_rss_MB.append(rss_KB / 1024)  # Convert from KB to MB
        all_vsz_MB.append(vsz_KB / 1024)  # Convert from KB to MB
        all_latency_us.append(latency_us)

        # Compute average metrics for the current directory
        average_cpu = round(np.mean(cpu_perc), 2)
        average_rss_MB = round(np.mean(rss_KB) / 1024, 2)  # Convert from KB to MB
        average_vsz_MB = round(np.mean(vsz_KB) / 1024, 2)  # Convert from KB to MB
        average_latency_us = round(np.mean(latency_us), 2)

        # Print and write average metrics in CSV format
        csv_file.write(f"{directory.lstrip('./')};{average_cpu};{average_rss_MB};{average_vsz_MB};{average_latency_us}\n")
        if directory == directories[0]:  # Print column titles only once
            print("Directory;CPU;RSS_MB;VSZ_MB;Latency_us")
        print(f"{directory.lstrip('./')};{average_cpu};{average_rss_MB};{average_vsz_MB};{average_latency_us}")

# Plotting if --plot is passed
if plot:
    # Plot CPU usage for all directories
    plt.figure(figsize=(8, 5))
    for time_ms, cpu_perc, directory in zip(all_time_ms, all_cpu_perc, directories):
        plt.plot(time_ms, cpu_perc, label=f'CPU_{directory}')
    plt.xlabel('Time (ms)')
    plt.ylabel('CPU Percentage')
    plt.title('CPU Usage over Time')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Plot RSS memory for all directories
    plt.figure(figsize=(8, 5))
    for time_ms, rss_MB, directory in zip(all_time_ms, all_rss_MB, directories):
        plt.plot(time_ms, rss_MB, label=f'RSS_{directory}')
    plt.xlabel('Time (ms)')
    plt.ylabel('RSS Memory (MB)')
    plt.title('RSS Memory over Time')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Plot VSZ memory for all directories
    plt.figure(figsize=(8, 5))
    for time_ms, vsz_MB, directory in zip(all_time_ms, all_vsz_MB, directories):
        plt.plot(time_ms, vsz_MB, label=f'VSZ_{directory}')
    plt.xlabel('Time (ms)')
    plt.ylabel('VSZ Memory (MB)')
    plt.title('VSZ Memory over Time')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Plot Latency for all directories
    plt.figure(figsize=(8, 5))
    for time_ms, latency_us, directory in zip(all_time_ms, all_latency_us, directories):
        plt.plot(time_ms, latency_us, label=f'Latency_{directory}')
    plt.xlabel('Time (ms)')
    plt.ylabel('Latency (us)')
    plt.title('Latency over Time')
    plt.grid(True)
    plt.legend()
    plt.show()
