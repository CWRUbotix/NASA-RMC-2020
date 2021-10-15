#!/usr/bin/python3
import atexit
import subprocess
import os
import datetime
import getpass

import rospy


class RosBagger():
    def __init__(self):
        rospy.init_node('bagging_node')

        bag_topics = rospy.get_param("~topics")

        today = datetime.datetime.now().strftime("%Y_%m_%d")
        now = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        # default bag location
        desired_path="$HOME" 

        # Will now store bags with respect to ROS_LOG_DIR if it has been set
        # Bags to external USB stick if the ROS_LOG_DIR is set to the path of the stick
        if 'ROS_LOG_DIR' in os.environ:
            desired_path =os.getenv('ROS_LOG_DIR')

            #gets parent directory of ros logs
            desired_path= os.path.abspath(os.path.join(desired_path , "../"))
       
        bag_dir = desired_path + "/glenn_bags/" + today


        if not os.path.exists(bag_dir):
            os.makedirs(bag_dir)

        symlink_dir =desired_path + "/glenn_bags/latest"

        # Record specific bag topics, save bag to bag_dir
        self.bag_process = subprocess.Popen(["rosbag", "record", "-O", bag_dir + "/" + now + ".bag"] + bag_topics)
        
        #creates link for ease of acess
        atexit.register(os.symlink,bag_dir + "/"+now + ".bag",symlink_dir)