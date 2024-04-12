import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def process_directory(directory):
    file_path = os.path.join(directory, "latency_all.txt")

    # Read the file
    with open(file_path, "r") as file:
        lines = file.readlines()

    # Find the start index of subscription and publisher sections
    sub_start = lines.index("Subscriptions stats:\n") + 1
    pub_start = lines.index("Publishers stats:\n") + 1

    # Find the end index of subscription and publisher sections
    sub_end = next((i for i, line in enumerate(lines[sub_start:], start=sub_start) if line.startswith("Publishers stats:") or line == "\n"), len(lines))
    pub_end = len(lines)

    # Read subscriptions data into a pandas DataFrame
    subscriptions_data = pd.read_csv(
        file_path,
        skiprows=sub_start,
        nrows=sub_end - sub_start - 1,  # Skip the last row containing "Publishers stats:" or the next section
        skipinitialspace=True
    )

    # Read publishers data into a pandas DataFrame
    publishers_data = pd.read_csv(
        file_path,
        skiprows=pub_start,
        nrows=pub_end - pub_start - 1,  # Exclude the last line containing a newline character
        skipinitialspace=True
    )

    # Find the position of "mean_us" and "sd_us" in subscription data
    sub_mean_col_index = subscriptions_data.columns.get_loc("mean_us")
    sub_std_col_index = subscriptions_data.columns.get_loc("sd_us")

    # Extract mean_us and sd_us values from subscriptions data
    sub_mean_values = subscriptions_data.iloc[:, sub_mean_col_index].tolist()
    sub_std_values = subscriptions_data.iloc[:, sub_std_col_index].tolist()
    sub_topics = subscriptions_data['topic'].tolist()

    # Calculate average latency and standard deviation for subscriptions
    sub_avg_latency = np.nanmean(sub_mean_values)
    sub_avg_std_dev = np.nanmean(sub_std_values)

    # Find the position of "mean_us" and "sd_us" in publisher data
    pub_mean_col_index = publishers_data.columns.get_loc("mean_us")
    pub_std_col_index = publishers_data.columns.get_loc("sd_us")

    # Extract mean_us and sd_us values from publisher data
    pub_mean_values = publishers_data.iloc[:, pub_mean_col_index].tolist()
    pub_std_values = publishers_data.iloc[:, pub_std_col_index].tolist()
    pub_topics = publishers_data['topic'].tolist()

    # Calculate average latency and standard deviation for publishers
    pub_avg_latency = np.nanmean(pub_mean_values)
    pub_avg_std_dev = np.nanmean(pub_std_values)

    return sub_mean_values, sub_std_values, sub_topics, sub_avg_latency, sub_avg_std_dev, pub_mean_values, pub_std_values, pub_topics, pub_avg_latency, pub_avg_std_dev


if len(sys.argv) < 2:
    print("Usage: python3 plot_latency.py <directory1> <directory2> ...")
    sys.exit(1)

directories = sys.argv[1:]

sub_mean_values_combined = []
sub_std_values_combined = []
sub_topics_combined = []
pub_mean_values_combined = []
pub_std_values_combined = []
pub_topics_combined = []

# Lists to store average values
avg_sub_latencies = []
avg_pub_latencies = []

# Process each directory
for directory in directories:
    sub_mean_values, sub_std_values, sub_topics, sub_avg_latency, sub_avg_std_dev, pub_mean_values, pub_std_values, pub_topics, pub_avg_latency, pub_avg_std_dev = process_directory(directory)

    sub_mean_values_combined.append(sub_mean_values)
    sub_std_values_combined.append(sub_std_values)
    sub_topics_combined.append(sub_topics)
    pub_mean_values_combined.append(pub_mean_values)
    pub_std_values_combined.append(pub_std_values)
    pub_topics_combined.append(pub_topics)

    # Append average latency values
    avg_sub_latencies.append(sub_avg_latency)
    avg_pub_latencies.append(pub_avg_latency)

    # Compute and print average metrics for the current directory
    print(f"Directory: {directory}")
    print("Average Latency for Subscriptions:", sub_avg_latency)
    print("Average Latency for Publishers:", pub_avg_latency)
    print()

# Print combined statistics in CSV format
print("Directory,Pub_avg_latency,Sub_avg_latency")
for directory, avg_pub_latency, avg_sub_latency in zip(directories, avg_pub_latencies, avg_sub_latencies):
    print(f"{directory},{avg_pub_latency},{avg_sub_latency}")

# Plot combined data
plt.figure(figsize=(10, 5))
for i, (sub_mean_values, sub_std_values, sub_topics) in enumerate(zip(sub_mean_values_combined, sub_std_values_combined, sub_topics_combined)):
    plt.errorbar(range(len(sub_mean_values)), sub_mean_values, yerr=sub_std_values, fmt='o', label=f'Subscriptions {directories[i]}')

    # Calculate and plot horizontal line for the average of mean_us
    # avg_mean_us_sub = np.mean(sub_mean_values)
    # plt.axhline(y=avg_mean_us_sub, color='r', linestyle='--', label=f'Avg Mean {directories[i]}: {avg_mean_us_sub:.2f} us')

for i, (pub_mean_values, pub_std_values, pub_topics) in enumerate(zip(pub_mean_values_combined, pub_std_values_combined, pub_topics_combined)):
    plt.errorbar(range(len(pub_mean_values)), pub_mean_values, yerr=pub_std_values, fmt='o', label=f'Publishers {directories[i]}')

    # Calculate and plot horizontal line for the average of mean_us
    # avg_mean_us_pub = np.mean(pub_mean_values)
    # plt.axhline(y=avg_mean_us_pub, color='g', linestyle='--', label=f'Avg Mean {directories[i]}: {avg_mean_us_pub:.2f} us')

plt.xticks(range(max(len(sub_mean_values_combined[0]), len(pub_mean_values_combined[0]))), sub_topics_combined[0], rotation=45, ha='right')
plt.xlabel('Index')
plt.ylabel('Latency (us)')
plt.title('Combined Latency Stats')
plt.legend()
plt.grid(True)
plt.show()
