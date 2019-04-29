## Create a bar plots.


import numpy as np
import csv
import sys
import os
import argparse
import re
from io import StringIO
from collections import defaultdict, OrderedDict

import matplotlib.pyplot
import matplotlib.ticker

import common
import cpu_ram_plot
import latency_reliability_plot


def parse_total_latency_csv(file_path):
    '''parses a csv into a dictionary structure, given its filepath'''

    # the input is not a csv so I have to convert it
    with open(file_path, 'r') as file:
        file_string = file.read()
        file.close()
    csv_file = re.sub('[ ]+', '\t', file_string)
    f = StringIO(csv_file)
    reader = csv.DictReader(f, delimiter='\t')

    data = {}

    for row_dict in reader:
        for key, value in row_dict.items():
            if not key:
                continue
            data[key] = float(value)

    return data


def main(argv):

    parser = argparse.ArgumentParser(description='Bar plots for evaluating benchmark app results')
    parser.add_argument('--resources', type=str, help='Pass the resources txt file')
    parser.add_argument('--latency', type=str, help='Pass the latency_total txt file')
    parser.add_argument('--target', type=str, help='Pass a target json file to evaluate current plot')
    parser.add_argument('--skip', type=int, default=2, help='Skip the first N seconds of each test')

    args = parser.parse_args()

    resources_path = args.resources
    latency_path = args.latency
    target = args.target
    skip = args.skip

    target_resources = common.parse_target_json(args.target, "resources")
    target_latency = common.parse_target_json(args.target, "latency_total")

    resources = cpu_ram_plot.parse_csv(resources_path, skip)
    average_resources = cpu_ram_plot.aggregate_csv(resources)
    latency = parse_total_latency_csv(latency_path)

    barWidth = 0.3

    # Create plots
    current_val = []
    target_val = []
    texts = []
    units = []

    for key, _ in target_resources.items():
        current_val.append(common.get_plot_data(average_resources, key)[0])
        target_val.append(common.get_plot_data(target_resources, key)[0])
        label = common.get_label(key)
        unit = common.get_unit_of_measure(label)
        texts.append(label)
        units.append(unit)


    for key, _ in target_latency.items():
        current_val.append(latency[key])
        target_val.append(target_latency[key])
        label = common.get_label(key)
        unit = common.get_unit_of_measure(label)
        texts.append(label)
        units.append(unit)


    fig, axs = matplotlib.pyplot.subplots(1, len(current_val), figsize=(11, 3))

    for i in range(len(current_val)):
        current_rect = axs[i].bar(0, current_val[i], barWidth, color='SkyBlue', label='current')[0]
        height_current = current_rect.get_height()
        axs[i].text(current_rect.get_x() + current_rect.get_width()/2.0, height_current, str(round(current_val[i], 2)), ha='center', va='bottom')

        target_rect = axs[i].bar(barWidth, target_val[i], barWidth, color='IndianRed', label='target')[0]
        height_target = target_rect.get_height()
        axs[i].text(target_rect.get_x() + target_rect.get_width()/2.0, height_target, str(round(target_val[i], 2)), ha='center', va='bottom')

        # Add some text for labels, title and custom x-axis tick labels, etc.
        if(current_val[i] > target_val[i]):
            color = "red"
        else:
            color = "green"

        axs[i].set_ylabel(units[i])
        axs[i].set_title(texts[i], color=color)
        if(i == 0):
            axs[i].legend(framealpha=0.5, loc="lower left", fontsize="small")
        axs[i].grid(axis='y')

        # Remove x ticks
        axs[i].tick_params(
            axis='x',          # changes apply to the x-axis
            which='both',      # both major and minor ticks are affected
            bottom=False,      # ticks along the bottom edge are off
            top=False,         # ticks along the top edge are off
            labelbottom=False) # labels along the bottom edge are off

    fig.tight_layout()
    matplotlib.pyplot.show()


if __name__ == '__main__':
    main(sys.argv[1:])