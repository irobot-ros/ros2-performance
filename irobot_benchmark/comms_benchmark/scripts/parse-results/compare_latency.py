import os
import csv
import sys
import time
import matplotlib.pyplot as plt

def get_sorted_files_by_mtime(directory):
    # Find all latency_all.txt files in the directory and sort them by modification time
    files = [os.path.join(root, file) for root, _, files in os.walk(directory) for file in files if file == 'latency_all.txt']
    return sorted(files, key=lambda f: os.path.getmtime(f))

def extract_sections(file_path):
    """
    Extract different sections from the latency_all.txt file.
    """
    with open(file_path, 'r') as f:
        lines = f.readlines()

    sections = {
        "subscriptions": [],
        "publishers": [],
        "clients": [],
        "services": [],
        "action_clients": [],
        "action_servers": []
    }

    current_section = None

    for line in lines:
        if "Action Clients stats:" in line:
            current_section = "action_clients"
            continue
        elif "Action Servers stats:" in line:
            current_section = "action_servers"
            continue
        elif "Clients stats:" in line:
            current_section = "clients"
            continue
        elif "Services stats:" in line:
            current_section = "services"
            continue
        elif "Subscriptions stats:" in line:
            current_section = "subscriptions"
            continue
        elif "Publishers stats:" in line:
            current_section = "publishers"
            continue

        if current_section and line.strip():
            sections[current_section].append(line)

    return sections

def calculate_average_from_section(lines):
    """
    Calculate the average mean_us value from a section's lines.
    """
    if not lines:
        return None

    reader = csv.DictReader(lines)
    mean_values = []
    for row in reader:
        mean_us = row.get('mean_us')
        if mean_us and mean_us.strip():
            try:
                mean_values.append(float(mean_us))
            except ValueError:
                print(f"Invalid 'mean_us' value: {mean_us}")
        else:
            print(f"Missing 'mean_us' in row: {row}")

    if mean_values:
        return sum(mean_values) / len(mean_values)
    return None

def process_directory(directory):
    """
    Process all latency_all.txt files in the directory and calculate the averages.
    """
    results = []
    # Get sorted latency_all.txt files by modification time
    sorted_files = get_sorted_files_by_mtime(directory)
    for file_path in sorted_files:
        if file_path.endswith('latency_all.txt'):
                root = os.path.dirname(file_path)  # Get root directory for cleaned path
                sections = extract_sections(file_path)

                avg_pub_duration = calculate_average_from_section(sections["publishers"])
                avg_sub_latency = calculate_average_from_section(sections["subscriptions"])
                avg_client_latency = calculate_average_from_section(sections["clients"])
                avg_service_latency = calculate_average_from_section(sections["services"])
                avg_action_client_latency = calculate_average_from_section(sections["action_clients"])
                avg_action_server_latency = calculate_average_from_section(sections["action_servers"])

                clean_root = root.lstrip("./")  # Remove leading "./" from the directory path
                results.append({
                    "Directory": clean_root,
                    "PubDur": avg_pub_duration,
                    "SubLat": avg_sub_latency,
                    "CliLat": avg_client_latency,
                    "SrvLat": avg_service_latency,
                    "ActionCliLat": avg_action_client_latency,
                    "ActionSrvLat": avg_action_server_latency
                })
    return results

def generate_average_latency_csv(results):
    """
    Generate a CSV file containing average latency data for all directories and print its content to the console.
    """
    with open("average_latency.csv", 'w', newline='') as csvfile:
        fieldnames = [
            'Directory', 'PubDur', 'SubLat',
            'CliLat', 'SrvLat',
            'ActionCliLat', 'ActionSrvLat'
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=';')
        writer.writeheader()

        # Print the header in the console
        print(";".join(fieldnames))

        for result in results:
            row = {
                'Directory': result["Directory"],
                'PubDur': round(result["PubDur"], 2) if result["PubDur"] is not None else '',
                'SubLat': round(result["SubLat"], 2) if result["SubLat"] is not None else '',
                'CliLat': round(result["CliLat"], 2) if result["CliLat"] is not None else '',
                'SrvLat': round(result["SrvLat"], 2) if result["SrvLat"] is not None else '',
                'ActionCliLat': round(result["ActionCliLat"], 2) if result["ActionCliLat"] is not None else '',
                'ActionSrvLat': round(result["ActionSrvLat"], 2) if result["ActionSrvLat"] is not None else ''
            }
            writer.writerow(row)

            # Print the row in the console
            print(";".join(str(row[field]) for field in fieldnames))

def plot_latency_metrics(results):
    """
    Plot the latency metrics for all directories in a single horizontal plot with different symbols.
    """
    directories = [result["Directory"] for result in results]
    metrics = {
        "PubDur": [result["PubDur"] or 0 for result in results],
        "SubLat": [result["SubLat"] or 0 for result in results],
        "CliLat": [result["CliLat"] or 0 for result in results],
        "SrvLat": [result["SrvLat"] or 0 for result in results],
        "ActionCliLat": [result["ActionCliLat"] or 0 for result in results],
        "ActionSrvLat": [result["ActionSrvLat"] or 0 for result in results],
    }

    # Assign each metric a unique color and symbol
    styles = {
        "PubDur": ("blue", "o"),  # Circle
        "SubLat": ("orange", "*"),  # Star
        "CliLat": ("green", "s"),  # Square
        "SrvLat": ("red", "D"),  # Diamond
        "ActionCliLat": ("purple", "^"),  # Triangle Up
        "ActionSrvLat": ("brown", "v"),  # Triangle Down
    }

    plt.figure(figsize=(12, 8))

    # Plot dots for each metric, skipping zero values
    for metric, values in metrics.items():
        color, marker = styles[metric]
        filtered_values = [val for val in values if val > 0]
        filtered_dirs = [directories[i] for i, val in enumerate(values) if val > 0]
        plt.scatter(filtered_values, filtered_dirs, label=metric, color=color, marker=marker, s=50)

    # Add labels and title
    plt.xlabel("Latency (us)")
    plt.ylabel("Directory")
    plt.title("Latency Metrics by Directory")
    plt.legend(title="Metrics", loc="upper right")
    plt.grid(True, axis="x", linestyle="--", alpha=0.7)
    plt.tight_layout()
    plt.show()


def print_help():
    """
    Print help and usage instructions.
    """
    print("""
Usage: python compare_latency.py <directory> [--plot]

Arguments:
  <directory>   Root directory to search for 'latency_all.txt' files.
  --plot        Optional flag to generate latency metric plots.

Description:
  This script processes all 'latency_all.txt' files in the specified directory (and its subdirectories),
  calculates average latency metrics (for publishers, subscriptions, clients, services, action clients, and
  action servers), and generates a CSV file (average_latency.csv) with these metrics.
  If the --plot flag is provided, it also generates plots for the metrics.

Example:
  python compare_latency.py ./my_directory --plot
""")
    sys.exit(0)

if __name__ == "__main__":
    if "--help" in sys.argv or "-h" in sys.argv:
        print_help()

    if len(sys.argv) < 2:
        print("Error: Missing required arguments.")
        print("Use --help or -h for usage instructions.")
        sys.exit(1)

    # Extract directory arguments and optional flags
    args = sys.argv[1:]
    plot = "--plot" in args
    directories = [arg for arg in args if arg != "--plot"]

    if not directories:
        print("Error: No directories provided.")
        sys.exit(1)

    all_results = []

    for directory in directories:
        if not os.path.isdir(directory):
            print(f"Warning: '{directory}' is not a valid directory. Skipping.")
            continue
        results = process_directory(directory)
        all_results.extend(results)

    if not all_results:
        print("No valid data found to process.")
        sys.exit(1)

    # Generate CSV
    generate_average_latency_csv(all_results)

    # Plot latency metrics if --plot is passed
    if plot:
        plot_latency_metrics(all_results)

