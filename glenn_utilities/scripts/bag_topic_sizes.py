#!/usr/bin/env python

"""
I found this script online from this helpful fellow
https://answers.ros.org/question/318667/using-rosbag-to-get-size-of-each-topic/

It tells you how much total space each topic takes ip
It may take a while to run depending on the bag size

Usage: rosrun glenn_utilities bag_topic_sizes.py path/to/bag
"""

import rosbag
import sys

topic_size_dict = {}

for topic, msg, time in rosbag.Bag(sys.argv[1], 'r').read_messages(raw=True):
    topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
topic_size = list(topic_size_dict.items())
topic_size.sort(key=lambda x: x[1])

size_names = ["B", "KB", "MB", "GB"]

for topic, size in topic_size:
    # Convert size in bytes to correct size and print
    for i, size_name in enumerate(size_names):
        if size < 10**(3 * (i + 1)):
            print(topic + ": " + "{:.1f}".format(size * 10**(-3 * i)) + size_names[i])
            break
