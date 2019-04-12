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
from collections import defaultdict, OrderedDict

import matplotlib.pyplot
import matplotlib.ticker

import common

def parse_csv(file_path, skip):
    '''parses a csv into a dictionary structure, given its filepath'''

    # get the basename of the directory where this csv is located
    dir_name = os.path.basename(os.path.dirname(file_path))

    # create an empty structure as starting point
    data = {
        'directory': dir_name,
        'cpu' : [],
        'time': [],
        'rss' : [],
        'vsz' : [],
        'pubs': 0,
        'subs': 0,
        'msg_type': '',
        'send_frequency': -1,
        'num_experiments': 1
    }

    print (file_path)

    reader = csv.DictReader(open(file_path), delimiter='\t')

    for i, row in enumerate(reader):

        # used to skip the first seconds if requested
        time = int(row['time'])
        if time < skip:
            continue

        data['time'].append(time)
        data['cpu'].append(float(row['cpu']))
        data['rss'].append(int(row['rss']))
        data['vsz'].append(int(row['vsz']))
        data['pubs'] = int(row['pubs'])
        data['subs'] = int(row['subs'])
        data['msg_type'] = row['msg_type']
        data['send_frequency'] = int(row['pub_freq'])

    return data


def split_csv(data):
    ''' given a parsed csv as a dictionary,
    it is splitted in many dictionaries each of them corresponding to 1 line of the original csv.'''

    num_output_files = len(data['time'])

    res = []

    for i in range(num_output_files):
        element_dict = {
            'directory' : data['directory'],
            'cpu' : data['cpu'][i],
            'time': data['time'][i],
            'rss' : data['rss'][i],
            'vsz' : data['vsz'][i],
            'pubs': data['pubs'],
            'subs': data['subs'],
            'msg_type': data['msg_type'],
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
    data['cpu'].pop(0)
    data['rss'].pop(0)
    data['vsz'].pop(0)

    # this is the number of measurements performed
    num_experiments = len(data['time'])

    res = {
        'directory' : data['directory'],
        'time': len(data['time']),
        'cpu' : sum(data['cpu']),
        'rss' : sum(data['rss']),
        'vsz' : sum(data['vsz']),
        'pubs': data['pubs'],
        'subs': data['subs'],
        'msg_type': data['msg_type'],
        'send_frequency': data['send_frequency'],
        'num_experiments': num_experiments
    }

    return res


def main(argv):

    parser = argparse.ArgumentParser(description='Plot results of performance test: [cpu,ram] vs [time]')
    parser.add_argument('dir_paths', nargs='+', default="", help='path to the directory containing the scripts we want to plot')
    parser.add_argument('--x', type=str,default='time', choices=['time', 'pubs', 'subs', 'msg_type'], help='value to display on x axis')
    parser.add_argument('--y', type=str, nargs='+', default=["cpu"], choices=['cpu', 'rss', 'vsz'], help='value to display on y axis')
    parser.add_argument('--y2', type=str, nargs='+', default=[], choices=['cpu', 'rss', 'vsz'], help='value to display on an additional y axis')
    parser.add_argument('--separator', nargs='+', default=[], choices=['send_frequency', 'directory', 'pubs', 'subs'], help='if not set all data are aggregated together, else aggregates only data which have the same value for the separator keys')
    parser.add_argument('--skip', type=int, default=0, help='Skip the first N seconds of each test')

    args = parser.parse_args()

    dir_paths = args.dir_paths
    x_key = args.x
    y1_keys = args.y
    y2_keys = args.y2
    separator = args.separator
    skip = args.skip

    __UNCOUNTABLE_DATA__ = ['time', 'send_frequency', 'msg_type', 'pubs', 'subs', 'separator']

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

    common.plot_function(data, x_key, y1_keys, y2_keys, separator)


if __name__ == '__main__':
    main(sys.argv[1:])

