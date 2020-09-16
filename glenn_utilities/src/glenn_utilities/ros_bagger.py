#!/usr/bin/python3
import subprocess
import os
import datetime

import rospy


class RosBagger():
    def __init__(self):
        rospy.init_node('bagging_node')

        bag_topics = rospy.get_param("~topics")
        bag_dir = os.path.expandvars("$HOME/glenn_bags")
        today = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

        if not os.path.exists(bag_dir):
            os.mkdir(bag_dir)

        # Record specific bag topics, save bag to bag_dir
        self.bag_process = subprocess.Popen(["rosbag", "record", "-O", bag_dir + "/" + today + ".bag"] + bag_topics)
