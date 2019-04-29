#!/usr/bin/env python

# collection of utilities used by both the plot scripts

import os
import sys
import re
import json
from collections import defaultdict, OrderedDict
import numpy as np
import matplotlib.pyplot
import matplotlib.ticker

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


def organize_data(data_samples, x_key, separator, uncountable_data):
    '''
    loops on data_samples list
    iteratively includes "new_data" into the "collected_data" dict.
    data are aggregated according to their value on the x_axis and to their separator value.
    e.g.    x_key = "subs"
            -> collected_data = {1: {values for 1 subs experiments}, 5: {values for 5 subs experiments}, ...}
    separator adds an additional level to the structure
    e.g.    separator = "msg_size", x_key = "subs"
            -> collected_data = {10b: {1:{values for 1 subs, 10b experiments}, 5:{values for 5 subs, 10b experiments}}, 100b {...}, ..}
    '''

    collected_data = defaultdict(dict)
    # if there are no separators, put all the data under the same key
    if not separator:
        collected_data['all'] = {}

    for new_data in data_samples:

        # get the x value of the new data, as we always use it
        x_val = new_data[x_key]

        if separator:
            # separators are a list of additional keys which divide the plots
            # e.g. separator=['spin_frequency'] will create different plots for each spin_frequency value found in the csv
            for separator_key in separator:
                # get the value for the separator_key
                separator_val = new_data[separator_key]
                # add the separator field to the data, to be able to retrieve it in the future (e.g. plot_function uses it)
                new_data['separator'] = separator_key
                # get the already collected data for this separator value
                plot_data = collected_data[separator_val]
                # merge the new data into the collected_separated ones at the position indicated by x_val
                plot_data[x_val] = merge_dictionaries(new_data, plot_data.get(x_val, {}), uncountable_data)
        else:
            # there is no separator so merge all data together in the first element of data dict (it is a dummy "all" label)
            plot_data = next(iter(collected_data.values()))
            plot_data[x_val] = merge_dictionaries(new_data, plot_data.get(x_val, {}), uncountable_data)

    return collected_data


def get_ax_ticks(key, min_v, max_v):
    '''given a range [min_v, max_v] of values of type "key", find a suitable number of axis ticks to display them '''

    # define custom ticks for some keys
    switcher = {
        'pubs': np.arange(min_v, max_v + 1.0, 1.0),
        'subs': np.arange(min_v, max_v + 1.0, 1.0)
    }

    # default ticks for all the other keys
    l = matplotlib.ticker.AutoLocator()
    l.create_dummy_axis()
    default_ticks = l.tick_values(min_v, max_v)
    # approximate default ticks at the 3rd decimal digit
    default_ticks = [round(x, 3) for x in default_ticks]
    # remove duplicate elements from default ticks for better visualization
    default_ticks = list(OrderedDict.fromkeys(default_ticks))

    return switcher.get(key, default_ticks)


def get_label(key):
    '''Convert a command line key key into a human readable label'''

    # if the provided key contains the unit of measure, remove it
    key = remove_unit_of_measure(key)

    switcher = {
        'pubs' : 'Publisher nodes [#]',
        'subs' : 'Subscriber nodes [#]',
        'reliability_sub': 'Reliability subscriber [%]',
        'reliability_pub': 'Reliability publisher  [%]',
        'reliability_tot': 'Total Reliability [%]',
        'lost': "Lost msgs [%]",
        'max_frequency': 'Maximum Frequency [Hz]',
        'latency': 'Latency [us]',
        'late': "Late msgs [%]",
        'too_late': "Too late msgs [%]",
        'msg_size': 'Message Size [KB]',
        'msg_type': 'Message Size [KB]',
        'time': 'Time [ms]',
        'rss': 'Physical RAM (RSS) [Mb]',
        'vsz': 'Virtual RAM (VSZ) [Mb]',
        'cpu': 'CPU usage [%]'
    }

    label = switcher.get(key)

    if not label:
        raise ValueError("Key: %s not found in `get_label` dictionary" % (key))

    return label


def get_title(x_key, y1_keys, y2_keys, separator):

    title = " ".join(y1_keys)
    if y2_keys:
        title +=', ' + " ".join(y2_keys)

    title += ' vs ' + x_key

    for i, sep in enumerate(separator):
        if sep == 'directory':
            continue
        if i is 0:
            title += ' for different values of ' + sep
        else:
            title += ' and ' + sep

    return title


def get_plot_data(data, key):
    '''extracts datapoints from dictionary, according to specified keys'''

    # In the reliability, received_count is the sum of all msg received, sent_count is the number of msg published.
    # To compare them I need to multiply the second for the number of subscribers

    # TODO: note that the reliability computation makes sense only for pub/sub.
    # with srv sent_count would already contain the total number of requests received by the server

    # if the provided key contains the unit of measure, remove it
    key = remove_unit_of_measure(key)

    # this allows to pass single csv to this function
    if depth(data) == 1:
        data = {'0': data}

    switcher = {
        # general
        'pubs' : lambda data: [ values['pubs']   for _, values in sorted(data.items()) ],
        'subs' : lambda data: [ values['subs']   for _, values in sorted(data.items()) ],
        'msg_size' : lambda data: [ values['msg_size']/1024   for _, values in sorted(data.items()) ],
        # latency and reliability
        'reliability_sub': lambda data: [ 0 if values['subs'] == 0 else min(1.0,values['received_count']/(values['sent_count'] * values['subs']))  * 100   for _, values in sorted(data.items()) ],
        'reliability_pub': lambda data: [ min(1.0, (values['sent_count'] * values['subs'])/ values['th_count']) * 100   for _, values in sorted(data.items()) ],
        'reliability_tot': lambda data: [a*b/100 for a,b in zip(switcher['reliability_sub'](data), switcher['reliability_pub'](data))],
        'max_frequency': lambda data: [ min(values['sent_count'] / values['duration'], values['send_frequency'])   for _, values in sorted(data.items())  ],
        'latency': lambda data: [ values['latency']/values.get('num_experiments', 1)   for _, values in sorted(data.items()) ],
        # cpu and RAM
        'time' : lambda data: [ values['time']   for _, values in sorted(data.items()) ],
        'cpu'  : lambda data: [ values['cpu[%]']/values.get('num_experiments', 1)  for _, values in sorted(data.items()) ],
        'rss'  : lambda data: [ (values['rss[KB]']/ 1024)/values.get('num_experiments', 1)   for _, values in sorted(data.items()) ],
        'vsz'  : lambda data: [ (values['vsz[KB]']/ 1024)/values.get('num_experiments', 1)   for _, values in sorted(data.items()) ]
    }

    plot_data = switcher[key](data)

    return plot_data




def plot_function(data_dict, x_key, y1_keys, y2_keys, separator, target = {}):


    # prepare plot and x-y axes
    fig, ax1 = matplotlib.pyplot.subplots()
    ax1.tick_params('y', colors='k')
    ax1.get_yaxis().get_major_formatter().set_useOffset(False)
    y1_labels = set()

    # I need to store min/max values because in each axis I have to use the extreme values among all the collected csv
    x_min = y1_min = y2_min = sys.maxsize
    x_max = y1_max = y2_max = -sys.maxsize

    # create a second y axis sharing the current x axis
    if y2_keys:
        ax2 = ax1.twinx()
        ax2.tick_params('y', colors='k')
        ax2.get_yaxis().get_major_formatter().set_useOffset(False)
        y2_labels = set()


    # loop on "separated" plot data (if --separator is not set, there will be only one)
    for label, data in sorted(data_dict.items()):

        # plot data on the main axes x, y
        for y1_key in y1_keys:

            # extract the values from the data dict, according to the keys
            x = get_plot_data(data, x_key)
            y = get_plot_data(data, y1_key)


            # ensure that values are ordered
            # NOTE: both lists are ordered with respect to X values
            y = [_y for _,_y in sorted(zip(x,y))]
            x = sorted(x)

            # create a name for this plot
            plot_label = y1_key
            if separator:
                separator_key = next(iter(data.values()))['separator']
                if separator_key != 'directory':
                    # I don't want to print `bouncy_directory_cpu`, I prefer `bouncy_cpu`
                    plot_label = str(label) + '_' + separator_key + '_' + plot_label
                else:
                    plot_label = str(label) + '_' + plot_label


            # plot the data
            ax1.plot(x, y, linestyle='-', label=plot_label)

            # update ticks for x, y axes
            x_min = min(x_min, min(x))
            x_max = max(x_max, max(x))
            ax1.set_xticks(get_ax_ticks(x_key, x_min, x_max))
            y1_min = min(y1_min, min(y))
            y1_max = max(y1_max, max(y))
            ax1.set_yticks(get_ax_ticks(y1_key, y1_min, y1_max))

            # set label for x, y axes
            ax1.set_xlabel(get_label(x_key), color='k')
            y1_labels.add(get_label(y1_key))
            ax1.set_ylabel(" ".join(y1_labels), color='k')

        # plot data on the second y axis
        for y2_key in y2_keys:

            # extract the values from the data dict, according to the keys
            x = get_plot_data(data, x_key)
            y = get_plot_data(data, y2_key)


            # ensure that values are ordered
            # NOTE: both lists are ordered with respect to X values
            y = [_y for _,_y in sorted(zip(x,y))]
            x = sorted(x)

            # create a name for this plot
            plot_label = y2_key
            if separator:
                separator_key = next(iter(data.values()))['separator']
                if separator_key != 'directory':
                    # I don't want to print `bouncy_directory_cpu`, I prefer `bouncy_cpu`
                    plot_label = str(label) + '_' + separator_key + '_' + plot_label
                else:
                    plot_label = str(label) + '_' + plot_label

            # Create a fake line in ax1, to display legend properly
            fake_plot = ax1.plot(np.nan,linestyle='-', label=plot_label)
            next_color = fake_plot[0].get_color()
            # plot the data
            ax2.plot(x, y, linestyle='-', label=plot_label, color=next_color)

            # update ticks for y2 axis
            y2_min = min(y2_min, min(y))
            y2_max = max(y2_max, max(y))
            ax2.set_yticks(get_ax_ticks(y2_key, y2_min, y2_max))

            # set label for y2 axis
            y2_labels.add(get_label(y2_key))
            ax2.set_ylabel(" ".join(y2_labels), color='k')

    # draw target horizontal lines for each y key
    if target:
        for y1_key in y1_keys:
            if y1_key not in target:
                continue
            plot_label = y1_key + "_target"
            target_value = target[y1_key]
            # TODO: axhline function restarts the colors cycle, so I need fake_plot to select the correct color
            # Create a fake line in ax1, to get next color
            fake_plot = ax1.plot(np.nan,linestyle='-', label=plot_label)
            next_color = fake_plot[0].get_color()
            # plot the data
            ax1.axhline(y=target_value, linestyle='-', label=plot_label, color=next_color)

            # update ticks for y1 axis
            y1_min = min(y1_min, target_value)
            y1_max = max(y1_max, target_value)
            ax1.set_yticks(get_ax_ticks(y1_key, y1_min, y1_max))

        for y2_key in y2_keys:
            if y2_key not in target:
                continue
            plot_label = y2_key + "_target"
            target_value = target[y2_key]
            # Create a fake line in ax1, to display legend properly and get next color
            fake_plot = ax1.plot(np.nan,linestyle='-', label=plot_label)
            next_color = fake_plot[0].get_color()
            # plot the data
            ax2.axhline(y=target_value, linestyle='-', label=plot_label, color=next_color)

            # update ticks for y2 axis
            y2_min = min(y2_min, target_value)
            y2_max = max(y2_max, target_value)
            ax2.set_yticks(get_ax_ticks(y2_key, y2_min, y2_max))


    # add a title to this plot
    matplotlib.pyplot.title(get_title(x_key, y1_keys, y2_keys, separator))

    # display legend in the plot
    ax1.legend(loc='upper left')

    fig.tight_layout()
    matplotlib.pyplot.show()
