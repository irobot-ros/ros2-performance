#!/usr/bin/env python

# Plot the results of run_and_monitor.sh script, i.e. memory and CPU usage of the
# ros2 process. It takes as input the file created by ./run_and_monitor.sh .
# The CPU usage is the average of CPU used since the process is created (that's
# the way ps works). This script also estimates the instantaneous CPU usage, so
# it could miss a spike on CPU usage.
# Usage example:
# python3 cpu_ram_plot.py <RESULT_DIRECTORY>

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

def parse_csv(file_path, skip = 0):
    '''parses a csv into a dictionary structure, given its filepath'''

    # get the basename of the directory where this csv is located
    dir_name = os.path.basename(os.path.dirname(file_path))

    # create an empty structure as starting point
    data = {
        'directory': dir_name,
        'cpu[%]' : [],
        'time': [],
        'rss[KB]' : [],
        'vsz[KB]' : [],
        'pubs': 0,
        'subs': 0,
        'msg_size[KB]': '',
        'send_frequency': -1,
        'num_experiments': 1
    }

    # the input is not a csv so I have to convert it
    with open(file_path, 'r') as file:
        file_string = file.read()
    csv_file = re.sub('[ ]+', '\t', file_string)
    f = StringIO(csv_file)
    reader = csv.DictReader(f, delimiter='\t')

    for i, row_dict in enumerate(reader):

        # used to skip the first seconds if requested
        time = int(row_dict['time[ms]']) + 1
        if time < skip:
            continue

        data['time'].append(time)
        data['cpu[%]'].append(float(row_dict.get('cpu[%]', 0)))
        data['rss[KB]'].append(int(row_dict.get('rss[KB]', 0)))
        data['vsz[KB]'].append(int(row_dict.get('vsz[KB]', 0)))
        data['pubs'] = int(row_dict.get('pubs', 0))
        data['subs'] = int(row_dict.get('subs', 0))
        data['msg_size[KB]'] = int(row_dict.get('msg_size[KB]', 0))
        data['send_frequency'] = int(row_dict.get('pub_freq', 0))

    return data


def split_csv(data):
    ''' given a parsed csv as a dictionary,
    it is splitted in many dictionaries each of them corresponding to 1 line of the original csv.'''

    num_output_files = len(data['time'])

    res = []

    for i in range(num_output_files):
        element_dict = {
            'directory' : data['directory'],
            'cpu[%]' : data['cpu[%]'][i],
            'time': data['time'][i],
            'rss[KB]' : data['rss[KB]'][i],
            'vsz[KB]' : data['vsz[KB]'][i],
            'pubs': data['pubs'],
            'subs': data['subs'],
            'msg_size[KB]': data['msg_size[KB]'],
            'send_frequency': data['send_frequency'],
            'num_experiments': data['num_experiments']  # is this value always 1?
        }

        res.append(element_dict)

    return res

def aggregate_csv(data):
    ''' given a parsed csv as a dictionary,
    it aggregates all its lines.'''

    # skip the first line of each element, since it is a "zero data line"
    # it is created in the run_and_monitor.sh script and it is only useful for time visualization
    # TODO: maybe this line should be only added by thin python script when needed
    data['time'].pop(0)
    data['cpu[%]'].pop(0)
    data['rss[KB]'].pop(0)
    data['vsz[KB]'].pop(0)

    # this is the number of measurements performed
    num_experiments = len(data['time'])

    res = {
        'directory' : data['directory'],
        'time': len(data['time']),
        'cpu[%]' : sum(data['cpu[%]']),
        'rss[KB]' : sum(data['rss[KB]']),
        'vsz[KB]' : sum(data['vsz[KB]']),
        'pubs': data['pubs'],
        'subs': data['subs'],
        'msg_size[KB]': data['msg_size[KB]'],
        'send_frequency': data['send_frequency'],
        'num_experiments': num_experiments
    }

    return res

# TODO: this is a quick fix to use the correct names
# a permanent solution would be to always use the full names (e.g. cpu[%] instead of cpu)
# immediately applying these fixes to the user input Command Line arguments
def fix_target_names(data):

    fixed_data = {}

    if 'time' in data:
        fixed_data['time'] = data['time']
    if 'cpu[%]' in data:
        fixed_data['cpu'] = data['cpu[%]']
    # TODO: I should apply the same conversion functions I use for plotting
    if 'rss[KB]' in data:
        fixed_data['rss'] = data['rss[KB]']/ 1000
    if 'vsz[KB]' in data:
        fixed_data['vsz'] = data['vsz[KB]']/ 1000

    return fixed_data

def main(argv):

    parser = argparse.ArgumentParser(description='Plot results of performance test: [cpu,ram] vs [time]')
    parser.add_argument('dir_paths', nargs='+', default="", help='path to the directory containing the scripts we want to plot')
    parser.add_argument('--x', type=str,default='time', choices=['time', 'pubs', 'subs', 'msg_size'], help='value to display on x axis')
    parser.add_argument('--y', type=str, nargs='+', default=["cpu"], choices=['cpu', 'rss', 'vsz'], help='value to display on y axis')
    parser.add_argument('--y2', type=str, nargs='+', default=[], choices=['cpu', 'rss', 'vsz'], help='value to display on an additional y axis')
    parser.add_argument('--separator', nargs='+', default=[], choices=['send_frequency', 'directory', 'pubs', 'subs'], help='if not set all data are aggregated together, else aggregates only data which have the same value for the separator keys')
    parser.add_argument('--target', type=str, default="", help='Pass a target json file to evaluate current plot')
    parser.add_argument('--skip', type=int, default=0, help='Skip the first N seconds of each test')

    args = parser.parse_args()

    dir_paths = args.dir_paths
    x_key = args.x
    y1_keys = args.y
    y2_keys = args.y2
    separator = args.separator
    skip = args.skip

    __UNCOUNTABLE_DATA__ = ['time', 'send_frequency', 'msg_size', 'pubs', 'subs', 'separator']

    # Get all files in folders in alphabetic order
    list_dir = common.get_files_from_paths(dir_paths)

    parsed_list = []
    # Collect data from csv files
    for file_path in list_dir:

        parsed_csv = parse_csv(file_path, skip)

        # given a parsed_csv, I can split all its lines into several csv or average them
        if x_key == "time":
            parsed_data = split_csv(parsed_csv)
        else:
            # TODO: i'm transforming it into a list to be able to use the for loop in the next line.. change it..
            parsed_data = [aggregate_csv(parsed_csv)]

        for dict_ in parsed_data:
            parsed_list.append(dict_)


    data = common.organize_data(parsed_list, x_key, separator, __UNCOUNTABLE_DATA__)

    parsed_target_json = {}
    if args.target:
        parsed_target_json = common.parse_target_json(args.target, "resources")
        parsed_target_json = fix_target_names(parsed_target_json)

    common.plot_function(data, x_key, y1_keys, y2_keys, separator, parsed_target_json)


if __name__ == '__main__':
    main(sys.argv[1:])

