import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def process_directory(directory):
    file_path = os.path.join(directory, "resources.txt")

    # Read the file
    resources_data = pd.read_csv(file_path)

    # Extract CPU percentage, RSS memory, and VSZ memory
    cpu_perc = resources_data['cpu_perc'].astype(float)
    rss_KB = resources_data['rss_KB'].astype(float)
    vsz_KB = resources_data['vsz_KB'].astype(float)
    time_ms = resources_data['time_ms']

    return time_ms, cpu_perc, rss_KB, vsz_KB

if len(sys.argv) < 2:
    print("Usage: python3 plot_metrics.py <directory1> <directory2> ...")
    sys.exit(1)

directories = sys.argv[1:]

# Lists to store data from all directories
all_time_ms = []
all_cpu_perc = []
all_rss_MB = []
all_vsz_MB = []

# CSV output file
csv_output_file = "average_metrics.csv"

# Process each directory and collect data
with open(csv_output_file, 'w') as csv_file:
    csv_file.write("Directory,Average_CPU,Average_RSS_MB,Average_VSZ_MB\n")

    for directory in directories:
        time_ms, cpu_perc, rss_KB, vsz_KB = process_directory(directory)
        all_time_ms.append(time_ms)
        all_cpu_perc.append(cpu_perc)
        all_rss_MB.append(rss_KB / 1024)  # Convert from KB to MB
        all_vsz_MB.append(vsz_KB / 1024)  # Convert from KB to MB

        # Compute average metrics for the current directory
        average_cpu = round(np.mean(cpu_perc), 2)
        average_rss_MB = round(np.mean(rss_KB) / 1024, 2)  # Convert from KB to MB
        average_vsz_MB = round(np.mean(vsz_KB) / 1024, 2)  # Convert from KB to MB

        # Print and write average metrics in CSV format
        csv_file.write(f"{directory},{average_cpu},{average_rss_MB},{average_vsz_MB}\n")
        if directory == directories[0]:  # Print column titles only once
            print("Directory,Average_CPU,Average_RSS_MB,Average_VSZ_MB")
        print(f"{directory},{average_cpu},{average_rss_MB},{average_vsz_MB}")

# Plot CPU usage for all directories
plt.figure(figsize=(8, 5))
for time_ms, cpu_perc, directory in zip(all_time_ms, all_cpu_perc, directories):
    plt.plot(time_ms.values, cpu_perc, label=f'CPU_{directory}')
plt.xlabel('Time (ms)')
plt.ylabel('CPU Percentage')
plt.title('CPU Usage over Time')
plt.grid(True)
plt.legend()
plt.show()

# Plot RSS memory for all directories
plt.figure(figsize=(8, 5))
for time_ms, rss_MB, directory in zip(all_time_ms, all_rss_MB, directories):
    plt.plot(time_ms.values, rss_MB, label=f'RSS_{directory}')
plt.xlabel('Time (ms)')
plt.ylabel('RSS Memory (MB)')
plt.title('RSS Memory over Time')
plt.grid(True)
plt.legend()
plt.show()

# Plot VSZ memory for all directories
plt.figure(figsize=(8, 5))
for time_ms, vsz_MB, directory in zip(all_time_ms, all_vsz_MB, directories):
    plt.plot(time_ms.values, vsz_MB, label=f'VSZ_{directory}')
plt.xlabel('Time (ms)')
plt.ylabel('VSZ Memory (MB)')
plt.title('VSZ Memory over Time')
plt.grid(True)
plt.legend()
plt.show()
