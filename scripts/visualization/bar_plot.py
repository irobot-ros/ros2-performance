 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2019, iRobot ROS
 #  All rights reserved.
 #
 #  This file is part of ros2-performance, which is released under BSD-3-Clause.
 #  You may use, distribute and modify this code under the BSD-3-Clause license.
 #

## Create a bar plots.
## You have to manually enter your data points into the values matrix.
## Divide it into multiple rows if you want multiple groups https://python-graph-gallery.com/11-grouped-barplot/
## Fill also a stacked_values matrix if you want stacked plots https://matplotlib.org/gallery/lines_bars_and_markers/bar_stacked.html


import numpy as np
import matplotlib.pyplot as plt

## NxM Values matrix: you have N groups and M types (each group will contain 1 element of each type)
## Each row is a group, each column contains all the elements of a same type
## (i.e. first element of each group)
## Position [0,0] contains the first value of the first group
## Position [0,2] contains the third value of the first group
## Position [1,0] contains the first value of the second group

values = np.array([
    [10, 10, 150],
    [10, 40, 200],
    [0, 100, 600]
])

stacked_values = np.array([
    [0, 0, 0],
    [300, 12028, 0],
    [0, 26000, 4000]
])

group_labels = ["1 pub 5 subs","1 pub 10 subs", "1 pub 15 subs"]
type_labels = ["Bouncy", "Crystal v1", "Master"]
y_label = "Time [ms]"

use_x_log_scale = False
use_y_log_scale = True

barWidth = 0.35
group_separator = 0.5

#########
######### here starts the plotting function
#########


num_groups = values.shape[0]
num_types = values.shape[1]
group_width = barWidth * num_groups


# the x locations for the center of the bars
# each element of rs has the positions of all the values of a type
rs = []
for i in np.arange(num_types):
    if i == 0:
        this_r = np.arange(num_groups) * (group_width + group_separator)
    else:
        previous_r = rs[i - 1]
        this_r = [x + barWidth for x in previous_r]

    rs.append(this_r)


# Make the plot
for i in range(values.shape[1]):
    plt.bar(rs[i], values[:,i],  width=barWidth, edgecolor='white', label=type_labels[i])

# Make the plot
for i in range(stacked_values.shape[1]):
    plt.bar(rs[i], stacked_values[:,i], bottom=values[:,i], color = "black", width=barWidth, edgecolor='white')


# Add xticks on the middle of the group bars
# Bars are |---|---|---|---|
# r are      0   1   3   4
# r starts half bar on the right because if I draw a bar at 1, it is centered at 1
plt.xticks([r +  barWidth*(num_types/2 - 0.5) for r in rs[0]], group_labels)

if use_x_log_scale:
    plt.xscale("log")
    plt.xlim(bottom=1)

if use_y_log_scale:
    plt.yscale("log")
    plt.ylim(bottom=1)


plt.ylabel(y_label)

# Create legend & Show graphic
plt.legend()
plt.show()
