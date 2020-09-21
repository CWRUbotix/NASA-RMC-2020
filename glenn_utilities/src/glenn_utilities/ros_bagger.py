#!/usr/bin/python3
import subprocess
import os
import datetime

import rospy


class RosBagger():
    def __init__(self):
        rospy.init_node('bagging_node')

        bag_topics = rospy.get_param("~topics")

        today = datetime.datetime.now().strftime("%Y_%m_%d")
        now = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        bag_dir = os.path.expandvars("$HOME/glenn_bags" + "/" + today)

        if not os.path.exists(bag_dir):
            os.makedirs(bag_dir)

        # Record specific bag topics, save bag to bag_dir
        self.bag_process = subprocess.Popen(["rosbag", "record", "-O", bag_dir + "/" + now + ".bag"] + bag_topics)
