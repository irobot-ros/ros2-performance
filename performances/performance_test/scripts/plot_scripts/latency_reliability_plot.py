#!/usr/bin/env python

# Plot the aggregated results of multinode statistics collected during several experiments.
# Communication latency and reliability are plotted.
# For subscriber nodes latency is computed as the difference between a message timestamp
# and the time when it's received. For clients it's the time between when a request is issued and when a response
# is received.
# Reliability_sub is computed as the percentage of received messages/response wrt their total number.
# Reliability_pub is computed as the percentage of sent msgs/total msgs.
# Reliability_tot is the system reliability, computed as (rel_pub * rel_sub).
# Usage example:
# python3 ros_performance_plot.py <OUTPUT_DIRECTORY> --x subs --y reliability_sub --y2 latency


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


def parse_csv(file_path):
    '''parses a csv into a dictionary structure, given its filepath'''

    # get the basename of the directory where this csv is located
    dir_name = os.path.basename(os.path.dirname(file_path))

    # create an empty structure as starting point
    data = {
        'directory': dir_name,      # number of nodes with at least a publisher or a server
        'pubs': 0,                  # number of nodes with at least a publisher or a server
        'subs': 0,                  # number of nodes with at least a subscriber or a client
        'valid_subscribers' : 0,    # number of subscribers or clients with at least 1 msg or srv::response received
        'received_count' : 0,       # total number of msg or srv::response received
        'sent_count' : 0,           # total number of msg published or srv called  TODO: check how to store here srv called
        'th_count' : -1,            # theoretical number of msgs that should be published by the publisher
        'latency' : 0,              # total latency of msg or srv::response received
        'spin_frequency' : 0,       # subscribers or servers spin frequency
        'send_frequency' : 0,       # publishers or clients frequency
        'msg_size' : 0,             # type of exchanged messages
        'duration': 0               # total duration of the experiment in seconds
    }

    # helper variables needed to fill the dictionary
    pubs_ids = set()
    subs_ids = set()

    # TODO: is there a way to support multiple frequencies in the same csv? right now I check that they are all equal
    # TODO: the `received_count` variables should be divided per topic. The publisher which starts first will send more messages than the others.
    # moreover this will be required once there is support for different frequencies.
    # TODO: some values (spin and send frequencies, msg_size, duration) should be equal among all lines. The check has been removed now

    # the input is not a csv so I have to convert it
    with open(file_path, 'r') as file:
        file_string = file.read()
    csv_file = re.sub('[ ]+', '\t', file_string)
    f = StringIO(csv_file)
    reader = csv.DictReader(f, delimiter='\t')

    rows_number = 0
    for row_dict in reader:
        rows_number += 1

        # store the fields of the current csv row into variables, adjustind their data type
        node_name = row_dict['node']
        topic = row_dict['topic']
        msg_count = int(row_dict.get('received[#]', 0))
        spin_frequency = int(row_dict.get('spin_frequency', -1))
        avg_latency = float(row_dict.get('mean[us]', 0))
        send_frequency = int(row_dict.get('freq[hz]', 0))
        msg_size = int(row_dict.get('size[b]', 0))
        experiment_duration = int(row_dict.get('duration[s]', 0))

        data['msg_size'] = msg_size
        data['duration'] = experiment_duration
        pubs_ids.add(topic)

        if node_name == topic:
            # this row denotes a publisher or server
            data['sent_count'] += msg_count
            data['send_frequency'] = send_frequency
            data['th_count'] = send_frequency * experiment_duration
        else:
            # this row denotes a subscriber or client
            subs_ids.add(node_name)

            data['received_count'] += msg_count
            data['spin_frequency'] = spin_frequency
            if msg_count > 0:
                # this row denotes a valid subscriber or client (it has received at least 1 msg or srv::response)
                data['latency'] += avg_latency
                data['valid_subscribers'] += 1
                data['send_frequency'] = send_frequency
                data['th_count'] = send_frequency * experiment_duration

    if rows_number < 1:
        return {}

    data['pubs'] = len(pubs_ids)
    data['subs'] = len(subs_ids)

    # choose the most reliable source for sent_count.
    # if publishers are in a different csv, data['sent_count'] is 0, but the subscriber can still get the theoretical count
    # NOTE: this estimate will be wrong if the publisher was not able to achieve its requested publish frequency
    if data['sent_count'] == 0:
        data['sent_count'] = data['th_count']

    return data



def main(argv):

    parser = argparse.ArgumentParser()
    parser.add_argument('dir_paths', nargs='+', type=str, default="", help='path to the directory containing the scripts we want to plot')
    parser.add_argument('--x', type=str, required=True, choices=['pubs', 'subs', 'msg_size'], help='value to display on x axis')
    parser.add_argument('--y', type=str, nargs='+', required=True, choices=['latency', 'reliability_sub', 'reliability_pub', 'reliability_tot', 'max_frequency', 'msg_size'], help='value to display on y axis')
    parser.add_argument('--y2', type=str, nargs='+', default=[], choices=['latency', 'reliability_sub', 'reliability_pub', 'reliability_tot', 'max_frequency', 'msg_size'], help='value to display on an additional y axis')
    parser.add_argument('--separator', nargs='+', default=[], choices=['spin_frequency', 'send_frequency', 'msg_size', 'directory', 'duration', 'pubs', 'subs'], help='if not set all data are aggregated together, else aggregates only data which have the same value for the separator keys')

    args = parser.parse_args()

    dir_paths = args.dir_paths
    x_axis = args.x
    y1_axis = args.y
    y2_axis = args.y2
    separator = args.separator

    __UNCOUNTABLE_DATA__ = ['directory', 'pubs', 'subs', 'spin_frequency', 'send_frequency', 'msg_size', 'separator']

    # Get all files in folders in alphabetic order
    list_dir = common.get_files_from_paths(dir_paths)

    parsed_list = []
    # Collect data from csv files
    for file_path in list_dir:

        parsed_csv = parse_csv(file_path)

        if not parsed_csv:
            continue

        parsed_list.append(parsed_csv)


    data = common.organize_data(parsed_list, x_axis, separator, __UNCOUNTABLE_DATA__)

    common.plot_function(data, x_axis, y1_axis, y2_axis, separator)


if __name__ == '__main__':
    main(sys.argv[1:])

