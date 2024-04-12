import os
import sys
import csv
import matplotlib.pyplot as plt

def calculate_average_stddev(csv_file):
    with open(csv_file, 'r', newline='') as file:
        reader = csv.DictReader(file)
        mean_values = []
        for row in reader:
            mean_values.append(float(row['mean_us']))
        if mean_values:
            avg_mean = sum(mean_values) / len(mean_values)
            return avg_mean
        else:
            return None

def process_directory(directory):
    pub_duration_csv = os.path.join(directory, "pub_duration.csv")
    sub_latency_csv = os.path.join(directory, "sub_latency.csv")

    avg_pub_duration = None
    avg_sub_latency = None

    if os.path.exists(pub_duration_csv):
        avg_pub_duration = calculate_average_stddev(pub_duration_csv)

    if os.path.exists(sub_latency_csv):
        avg_sub_latency = calculate_average_stddev(sub_latency_csv)

    return avg_pub_duration, avg_sub_latency

def generate_average_latency_data(directories):
    pub_duration_data = []
    sub_latency_data = []
    for directory in directories:
        avg_pub_duration, avg_sub_latency = process_directory(directory)
        # Replace None values with zeros
        pub_duration_data.append(round(avg_pub_duration, 2) if avg_pub_duration is not None else 0)
        sub_latency_data.append(round(avg_sub_latency, 2) if avg_sub_latency is not None else 0)
    return pub_duration_data, sub_latency_data

def plot_publisher_latency(pub_duration_data, directories):
    plt.figure(figsize=(8, 5))
    plt.bar(directories, pub_duration_data, color='skyblue')
    plt.xlabel('Directory')
    plt.ylabel('AvgPubDuration')
    plt.title('Average Publishing Duration')
    plt.tight_layout()
    plt.show()

def plot_subscription_latency(sub_latency_data, directories):
    plt.figure(figsize=(8, 5))
    plt.bar(directories, sub_latency_data, color='lightgreen')
    plt.xlabel('Directory')
    plt.ylabel('AvgSubLatency')
    plt.title('Average Subscription Latency')
    plt.tight_layout()
    plt.show()

def generate_average_latency_csv(directories):
    with open("average_latency.csv", 'w', newline='') as csvfile:
        fieldnames = ['Directory', 'AvgPubDuration', 'AvgSubLatency']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for directory in directories:
            avg_pub_duration, avg_sub_latency = process_directory(directory)
            writer.writerow({
                'Directory': directory,
                'AvgPubDuration': round(avg_pub_duration, 2) if avg_pub_duration is not None else '',
                'AvgSubLatency': round(avg_sub_latency, 2) if avg_sub_latency is not None else ''
            })

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py directory1 directory2 ...")
        sys.exit(1)

    directories = sys.argv[1:]
    generate_average_latency_csv(directories)

    with open("average_latency.csv", 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            print(','.join(row))

    pub_duration_data, sub_latency_data = generate_average_latency_data(directories)
    plot_publisher_latency(pub_duration_data, directories)
    plot_subscription_latency(sub_latency_data, directories)
