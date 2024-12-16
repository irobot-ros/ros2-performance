#!/usr/bin/env python3

import sys
import os
import pandas as pd

def print_help():
    """Print usage and help information."""
    print("""
Usage: python3 compare_all_metrics.py <results_dir1> <results_dir2> [--threshold <percentage>]

Arguments:
  <results_dir1>       Path to the first results directory (must contain average_metrics.csv).
  <results_dir2>       Path to the second results directory (must contain average_metrics.csv).
  --threshold <number> Optional percentage threshold. Only metrics with differences greater than this threshold or less than the negative of it are displayed in the console.

Description:
  This script compares metrics between two results directories. It calculates the percentage change for each metric and generates a CSV file with the results. If a threshold is specified, metrics exceeding this difference (positively or negatively) are displayed in the console along with the detailed line diff and original values.

Example:
  python3 compare_all_metrics.py results1 results2
  python3 compare_all_metrics.py results1 results2 --threshold 50

Output:
  - A CSV file named 'comparison_metrics.csv' with all percentage differences.
  - Console output of metrics with differences exceeding the specified threshold (if provided).
""")
    sys.exit(0)

def calculate_percentage_change(value1, value2):
    """Calculate percentage change from value1 to value2."""
    if value1 == 0:
        return "Infinity" if value2 != 0 else "0"
    return ((value2 - value1) / value1) * 100

# Handle help flag
if "--help" in sys.argv or "-h" in sys.argv:
    print_help()

# Check command-line arguments
if len(sys.argv) < 3 or len(sys.argv) > 5:
    print("Error: Invalid arguments.")
    print("Use --help or -h for usage instructions.")
    sys.exit(1)

results_dir1 = sys.argv[1]
results_dir2 = sys.argv[2]

# Optional threshold
threshold = None
if "--threshold" in sys.argv:
    try:
        threshold_index = sys.argv.index("--threshold") + 1
        threshold = float(sys.argv[threshold_index])
    except (IndexError, ValueError):
        print("Error: Invalid threshold value. Provide a valid number after --threshold.")
        sys.exit(1)

# Paths to average_metrics.csv in both directories
metrics_file1 = os.path.join(results_dir1, "average_metrics.csv")
metrics_file2 = os.path.join(results_dir2, "average_metrics.csv")

if not os.path.exists(metrics_file1) or not os.path.exists(metrics_file2):
    print(f"Error: average_metrics.csv not found in one or both directories: {results_dir1}, {results_dir2}")
    sys.exit(1)

# Load the data
df1 = pd.read_csv(metrics_file1)
df2 = pd.read_csv(metrics_file2)

# Merge the two dataframes on the Directory column
merged = pd.merge(df1, df2, on="Directory", suffixes=("1", "2"))

# Calculate percentage change for each metric
merged["Average_CPU %"] = merged.apply(lambda row: round(calculate_percentage_change(row["Average_CPU1"], row["Average_CPU2"]), 2), axis=1)
merged["Average_RSS_MB %"] = merged.apply(lambda row: round(calculate_percentage_change(row["Average_RSS_MB1"], row["Average_RSS_MB2"]), 2), axis=1)
merged["Average_VSZ_MB %"] = merged.apply(lambda row: round(calculate_percentage_change(row["Average_VSZ_MB1"], row["Average_VSZ_MB2"]), 2), axis=1)

# Prepare the final result DataFrame
result = merged[["Directory", "Average_CPU %", "Average_RSS_MB %", "Average_VSZ_MB %"]].copy()

# Save the result to a CSV file
output_file = "comparison_metrics.csv"
result.to_csv(output_file, index=False)
print(f"Comparison completed. Output saved to {output_file}")

# Print metrics with differences exceeding the threshold
if threshold is not None:
    print(f"\nMetrics with percentage differences exceeding {threshold}% or below {-threshold}%:\n")
    filtered = merged[
        (merged["Average_CPU %"] > threshold) | (merged["Average_CPU %"] < -threshold) |
        (merged["Average_RSS_MB %"] > threshold) | (merged["Average_RSS_MB %"] < -threshold) |
        (merged["Average_VSZ_MB %"] > threshold) | (merged["Average_VSZ_MB %"] < -threshold)
    ]
    if not filtered.empty:
        for _, row in filtered.iterrows():
            print(f"Directory: {row['Directory']}")
            if abs(row["Average_CPU %"]) > threshold:
                print(f"  Average_CPU %: {row['Average_CPU %']}% (Val1={row['Average_CPU1']}, Val2={row['Average_CPU2']})")
            if abs(row["Average_RSS_MB %"]) > threshold:
                print(f"  Average_RSS_MB %: {row['Average_RSS_MB %']}% (Val1={row['Average_RSS_MB1']}, Val2={row['Average_RSS_MB2']})")
            if abs(row["Average_VSZ_MB %"]) > threshold:
                print(f"  Average_VSZ_MB %: {row['Average_VSZ_MB %']}% (Val1={row['Average_VSZ_MB1']}, Val2={row['Average_VSZ_MB2']})")
            print("-" * 40)
    else:
        print("No metrics exceeded the specified threshold.")
