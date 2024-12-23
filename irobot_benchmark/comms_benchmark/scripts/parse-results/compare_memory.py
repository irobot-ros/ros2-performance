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

def calculate_regressions(memory_data, ignore_first_n=0, num_samples=3):
    """
    Calculate linear regression for each dataset in memory_data.
    Ignore the first `ignore_first_n` samples and take the subsequent `num_samples` samples.
    If the total number of samples is fewer than `ignore_first_n + num_samples`, adjust accordingly.
    """
    regression_results = {}

    for file_name, data in memory_data.items():
        N, RSS = data['N'], data['RSS']

        if len(N) <= ignore_first_n:
            print(f"Skipping {file_name} due to insufficient data points after ignoring the first {ignore_first_n}.")
            continue

        N_reg = N[ignore_first_n:]
        RSS_reg = RSS[ignore_first_n:]

        if num_samples is not None:
            N_reg = N_reg[:num_samples]
            RSS_reg = RSS_reg[:num_samples]

        slope, intercept, r_value, _, std_err = linregress(N_reg, RSS_reg)
        regression_line = slope * N_reg + intercept

        regression_results[file_name] = {
            'slope': slope,
            'intercept': intercept,
            'r2': r_value**2,  # Return R²
        }

    return regression_results

def plot_memory_scaling(memory_data, regression_results, output_file, ignore_first_n_plot=2, omit_last_m_plot=1):
    """
    Plot RSS vs. N for each file in memory_data on separate subplots within the same canvas.
    Include linear regression lines for each file.
    Optionally ignore the first `ignore_first_n_plot` entries and omit the last `omit_last_m_plot` entries for plotting.
    """
    num_plots = len(memory_data)
    num_columns = 3
    num_rows = (num_plots + num_columns - 1) // num_columns

    fig, axes = plt.subplots(num_rows, num_columns, figsize=(15, 5 * num_rows))
    axes = axes.flatten()

    # Generate dynamic color pairs using a colormap
    color_map = get_cmap('tab10').colors
    color_pairs = list(itertools.combinations(color_map, 2))

    for idx, (file_name, data) in enumerate(memory_data.items()):
        N, RSS = data['N'], data['RSS']

        # Skip first `ignore_first_n_plot` entries
        if len(N) <= ignore_first_n_plot:
            print(f"Skipping {file_name} due to insufficient data points after ignoring the first {ignore_first_n_plot}.")
            continue

        N_plot = N[ignore_first_n_plot:]
        RSS_plot = RSS[ignore_first_n_plot:]

        # Further omit the last `omit_last_m_plot` items
        N_plot = N_plot[:-omit_last_m_plot] if omit_last_m_plot > 0 else N_plot
        RSS_plot = RSS_plot[:-omit_last_m_plot] if omit_last_m_plot > 0 else RSS_plot

        # Get regression results
        regression_result = regression_results.get(file_name)
        if regression_result:
            slope = regression_result['slope']
            intercept = regression_result['intercept']
            r2 = regression_result['r2']

            # Extend the regression line over the entire range of the plot
            N_extended = np.linspace(N_plot[0], N_plot[-1], 100)  # 100 points for smooth line
            regression_line_extended = slope * N_extended + intercept

            # Get color pair for the current plot
            data_color, fit_color = color_pairs[idx % len(color_pairs)]

            ax = axes[idx]
            # Plot the actual data
            ax.plot(N_plot, RSS_plot, 'o-', color=data_color, label=f'{file_name} (data)')

            # Plot the extended regression line
            ax.plot(N_extended, regression_line_extended, '--', color=fit_color, label=f'{file_name} (fit, R²={r2:.2f})')

            # Add labels, title, and legend
            ax.set_xlabel('Number of Entities (N)')
            ax.set_ylabel('RSS Memory (MB)')
            ax.set_title(f'RSS Memory Scaling for {file_name}')
            ax.legend(loc='upper left')
            ax.grid(True, linestyle='--', alpha=0.5)

    # Hide any unused subplots
    for idx in range(len(memory_data), len(axes)):
        fig.delaxes(axes[idx])

    plt.tight_layout()
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

    # Calculate regression results
    print("Calculating regressions...")
    regression_results = calculate_regressions(memory_data, ignore_first_n=3, num_samples=3)

    # Generate plot
    print("Generating memory_scaling_comparison.png...")
    output_plot = os.path.join(results_dir, "memory_scaling_comparison.png")
    plot_memory_scaling(memory_data, regression_results, output_plot, ignore_first_n_plot=0, omit_last_m_plot=1)

    print(f"Processing completed. Results saved in:\n  - {output_plot}")

if __name__ == "__main__":
    # Ensure the script is called with one argument
    if len(sys.argv) != 2:
        print("Usage: python3 script.py <results_directory>")
        sys.exit(1)

    results_dir = sys.argv[1]
    main(results_dir)