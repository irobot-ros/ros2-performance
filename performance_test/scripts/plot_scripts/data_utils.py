 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2019, iRobot ROS
 #  All rights reserved.
 #
 #  This file is part of ros2-performance, which is released under BSD-3-Clause.
 #  You may use, distribute and modify this code under the BSD-3-Clause license.
 #

#!/usr/bin/env python

# general utilities for files and data handling

import csv
import json
import os
import re
import sys

from io import StringIO

def get_files_from_paths(paths):
    '''returns a list of paths to files, given one or more directory or file paths.
        path files are sorted in ascending natural order'''

    def atoi(text):
        return int(text) if text.isdigit() else text

    def natural_keys(text):
        '''
        alist.sort(key=natural_keys) sorts in human order
        http://nedbatchelder.com/blog/200712/human_sorting.html
        (See Toothy's implementation in the comments)
        '''
        return [ atoi(c) for c in re.split(r'(\d+)', text) ]

    file_paths_list = []
    for path in paths:
        if os.path.isdir(path):
            this_dir_files = os.listdir(path)
            this_dir_paths = [os.path.join(path, filename) for filename in this_dir_files]
            file_paths_list += this_dir_paths
        elif os.path.isfile(path):
            file_paths_list.append(path)
        else:
            print(path, " is not a valid file or directory!")
            sys.exit(1)

    file_paths_list.sort(key=natural_keys)

    return file_paths_list


def parse_target_json(target_file, experiment_type = ""):
    '''experiment_type can be "resources" or "latency_total"'''

    json_data = open(target_file)
    parsed_json = json.load(json_data)

    if experiment_type:
        return parsed_json[experiment_type]

    merged_target = {**parsed_json["resources"], **parsed_json["latency_total"]}

    return merged_target


def parse_csv_dict(file_path, delimiter='\t'):
    '''parse a file converting it into a csv dictionary'''

    with open(file_path, 'r') as file:
        file_string = file.read()

    # the input may not be a proper csv so I have to convert it
    csv_file = re.sub('[ ]+', delimiter, file_string)
    f = StringIO(csv_file)

    reader = csv.DictReader(f, delimiter=delimiter)

    return reader


def merge_dictionaries(dict1, dict2, uncountable_data=[]):
    '''
    merge two dictionaries with the following logic
    dict1 = {'both1':1, 'both2':2, 'only_1': 100 }
    dict2 = {'both1':10, 'both2': 20, 'only_2':200 }
    ---->
    merged_dict = {'only_2': 200, 'both2': 22, 'both1': 11, 'only_1': 100}

    this method handles multiple datatypes, as long as they support the `+` operator

    the uncountable_data array is used to specify those fields which can't be summed together
    '''

    #TODO: throw an error if trying to mix dictionaries with different values of an uncountable data

    merged_dict = {}

    # insert all items from dict1 in the merged dictionary, checking also their presence in dict2
    for key, value in dict1.items():

        if key in uncountable_data:
            merged_dict[key] = value
        else:
            default_val = type(value)()
            merged_dict[key] = value + dict2.get(key, default_val)

    # insert all items from dict2 which are not present in dict1 in the merged dictionary
    for key, value in dict2.items():

        if key in uncountable_data:
            merged_dict[key] = value
        elif key not in dict1:
            merged_dict[key] = value


    return merged_dict


def depth(d, level=0):
    if not isinstance(d, dict) or not d:
        return level
    return max(depth(d[k], level + 1) for k in d)


def get_unit_of_measure(label):
    # NOTE: this assumes that unit of measures is within square brackets at the end of the key
    search_result = re.search('\[(.+)\]$', label)
    if search_result:
        return search_result.group(1)
    else:
        return ""


def remove_unit_of_measure(label):
    # NOTE: this assumes that unit of measures is within square brackets at the end of the key
    new_label = re.sub('\[(.+)\]$', '', label)
    return new_label