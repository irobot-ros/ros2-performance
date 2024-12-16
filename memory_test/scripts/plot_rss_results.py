#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import sys

# Get all CSV files passed as input arguments
files = sys.argv[1:]

# Initialize a plot
plt.figure(figsize=(10, 6))

# Define different line styles for groups of 5 files
line_styles = ['-', '--', ':']

# Iterate over all the CSV files
for idx, file in enumerate(files):
    # Read each CSV file
    df = pd.read_csv(file)
    
    # Extract the RSS column and convert it to MB
    rss_mb = df['RSS'] / 1024
    
    # Extract the filename without the extension for labeling
    label = f"{file.split('.')[0]} [{idx + 1}]"
    
    # Select line style based on the file index
    line_style = line_styles[(idx // 5) % len(line_styles)]
    
    # Plot the RSS values in MB with the file name as label
    plt.plot(rss_mb, label=label, linestyle=line_style)
    
    # Place the number on top of the last value of the line
    plt.text(len(rss_mb)-1, rss_mb.iloc[-1], f"[{idx + 1}]", fontsize=10, verticalalignment='bottom')

# Add horizontal lines for every 10 MB on the Y axis
y_min, y_max = plt.ylim()
plt.yticks(range(int(y_min), int(y_max) + 10, 10))
plt.grid(axis='y', linestyle='--', linewidth=0.7)

# Add labels and title
plt.xlabel('Position')
plt.ylabel('RSS [MB]')
plt.title('RSS Values (in MB) from Multiple CSV Files')
plt.legend(loc='best', fontsize='small', bbox_to_anchor=(1.05, 1))

# Display the plot
plt.tight_layout()
plt.show()

