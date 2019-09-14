#!/usr/bin/env python3
import os
import glob
import math
import rospy
import rospkg
import math
import matplotlib
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, TwistWithCovariance, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from hci.msg import sensorValue



class NodeStartup:
    def __init__(self):
        print('Booting up node...')
        rospy.init_node('node_startup_test', anonymous=True)
        print('Succesfully connected to ROS master')

    def msg_callback(msg):
        print(msg)


if __name__ == '__main__':
    node = NodeStartup()
    sub = rospy.Subscriber('sensorValue', sensorValue, node.msg_callback)
    rospy.spin()
