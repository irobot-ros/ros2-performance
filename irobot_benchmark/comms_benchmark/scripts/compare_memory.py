#!/usr/bin/env python3

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import itertools
from matplotlib.cm import get_cmap

def parse_memory_results(directory):
    """
    Parse all CSV files in the memory_results directory.
    Extracts columns N, RSS, and DELTA.
    """
    memory_data = {}

    # Iterate through all CSV files in the directory
    for file_name in os.listdir(directory):
        if file_name.endswith('.csv'):
            file_path = os.path.join(directory, file_name)

            # Read the CSV file
            df = pd.read_csv(file_path)

            # Extract N, RSS, and DELTA columns
            if all(col in df.columns for col in ['N', 'RSS', 'DELTA']):
                memory_data[file_name] = {
                    'N': df['N'].values,
                    'RSS': df['RSS'].values / 1024,  # Convert KB to MB
                    'DELTA': df['DELTA'].values,
                }
            else:
                print(f"Warning: Missing expected columns in {file_name}")

    return memory_data

def analyze_scalability(N, RSS):
    """
    Perform linear regression on N vs. RSS.
    Returns slope, intercept, standard error, and R².
    """
    slope, intercept, r_value, _, std_err = linregress(N, RSS)
    return slope, intercept, std_err, r_value**2  # Return R²

def plot_memory_scaling_with_regression(memory_data, output_file, ignore_first_n=2, omit_last_n=1):
    """
    Plot RSS vs. N for each file in memory_data on the same canvas.
    Include linear regression lines for each file.
    Optionally ignore the first `ignore_first_n` entries and omit the last `omit_last_n` entries.
    """
    plt.figure(figsize=(10, 6))  # Set canvas size

    # Generate dynamic color pairs using a colormap
    color_map = get_cmap('tab10').colors
    color_pairs = list(itertools.combinations(color_map, 2))

    for idx, (file_name, data) in enumerate(memory_data.items()):
        N, RSS = data['N'], data['RSS']

        # Skip first `ignore_first_n` entries
        if len(N) <= ignore_first_n:
            print(f"Skipping {file_name} due to insufficient data points after ignoring the first {ignore_first_n}.")
            continue

        N_subset = N[ignore_first_n:]
        RSS_subset = RSS[ignore_first_n:]

        # Further omit the last `omit_last_n` items
        N_subset = N_subset[:-omit_last_n] if omit_last_n > 0 else N_subset
        RSS_subset = RSS_subset[:-omit_last_n] if omit_last_n > 0 else RSS_subset

        # Perform linear regression
        slope, intercept, _, r2 = analyze_scalability(N_subset, RSS_subset)
        regression_line = slope * N_subset + intercept

        # Get color pair for the current plot
        data_color, fit_color = color_pairs[idx % len(color_pairs)]

        # Plot the actual data
        plt.plot(N[ignore_first_n:-omit_last_n if omit_last_n > 0 else None], 
                 RSS[ignore_first_n:-omit_last_n if omit_last_n > 0 else None],
                 'o-', color=data_color, label=f'{file_name} (data)')

        # Plot the regression line (only for N_subset range)
        plt.plot(N_subset, regression_line, '--', color=fit_color, label=f'{file_name} (fit, R²={r2:.2f})')

    # Add labels, title, and legend
    plt.xlabel('Number of Entities (N)')
    plt.ylabel('RSS Memory (MB)')
    plt.title('RSS Memory Scaling with Linear Regression')
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1), ncol=1)
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()

    # Save and show the plot
    plt.savefig(output_file, bbox_inches='tight')
    plt.show()

def main(results_dir):
    # Verify memory_results directory exists
    memory_results_dir = os.path.join(results_dir, "memory_results")
    if not os.path.exists(memory_results_dir):
        print(f"Error: Directory '{memory_results_dir}' not found.")
        sys.exit(1)

    # Parse memory_results
    print("Parsing memory results...")
    memory_data = parse_memory_results(memory_results_dir)

    # Generate plot
    print("Generating memory_scaling_comparison.png...")
    output_plot = os.path.join(results_dir, "memory_scaling_comparison.png")
    plot_memory_scaling_with_regression(memory_data, output_plot, ignore_first_n=0, omit_last_n=1)

    print(f"Processing completed. Results saved in:\n  - {output_plot}")

if __name__ == "__main__":
    # Ensure the script is called with one argument
    if len(sys.argv) != 2:
        print("Usage: python3 script.py <results_directory>")
        sys.exit(1)

    results_dir = sys.argv[1]
    main(results_dir)
