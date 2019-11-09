#!/usr/bin/env python3
import os
import glob
import math
import rospy
import rospkg
import math
import matplotlib
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from geometry_msgs.msg import Pose, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry


class TrajectoryPrediction:
    def __init__(self):
        self.odom_topic = 'odometry/filtered_map'
        self.accel_topic = 'accel/filtered'
        self.x = None
        self.y = None
        self.yaw = None
        self.x_vel = None
        self.y_vel = None
        self.yaw_vel = None
        self.x_accel = None
        self.y_accel = None
        self.yaw_accel = None
        self.old_timestamp = None
        self.new_timestamp = None
        print('Booting up node...')
        rospy.init_node('trajectory_prediction', anonymous=True)

    def full_state_available(self):
        return self.x is not None and self.y is not None and self.x_vel is not None and self.y_vel is not None and self.x_accel is not None and self.y_accel is not None

    def odom_callback(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        quat = pose.orientation
        euler = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
        if not self.full_state_available():  # first state message received
            self.old_timestamp = msg.header.stamp.to_nsec()
            self.new_timestamp = msg.header.stamp.to_nsec()
        else:  # new state message, predict and measure deviation
            self.new_timestamp = msg.header.stamp.to_nsec()
            t = (self.new_timestamp - self.old_timestamp) * 1e-9  # time elapsed between prediction and observation converted to seconds
            x_new = self.x + self.x_vel * t + 0.5 * self.x_accel * t ** 2
            y_new = self.y + self.y_vel * t + 0.5 * self.y_accel * t ** 2
            yaw_new = self.yaw + self.yaw_vel + 0.5 * self.yaw_accel * t ** 2
            x_vel_new = self.x_vel + self.x_accel * t
            y_vel_new = self.y_vel + self.y_accel * t
            yaw_vel_new = self.yaw_vel + self.yaw_accel * t

            x_error = abs(x_new - self.x)
            y_error = abs(y_new - self.y)
            yaw_error = abs(yaw_new - self.yaw)
            x_vel_error = abs(x_vel_new - self.x_vel)
            y_vel_error = abs(y_vel_new - self.y_vel)
            yaw_vel_error = abs(yaw_vel_new - self.yaw_vel)

            delta_l = x_vel_error / math.cos(self.yaw) - (self.yaw_vel * 0.63) / 2
            delta_r = x_vel_error / math.cos(self.yaw) + (self.yaw_vel * 0.63) / 2

            print(delta_r, delta_l)

        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = euler[2]
        self.x_vel = twist.linear.x
        self.y_vel = twist.linear.y
        self.yaw_vel = twist.angular.z
        self.old_timestamp = self.new_timestamp  # set timestamp to be used for prediction as current msg timestamp

    def accel_callback(self, msg):
        linear = msg.accel.accel.linear
        angular = msg.accel.accel.angular
        self.x_accel = linear.x
        self.y_accel = linear.y
        self.yaw_accel = angular.z


if __name__ == '__main__':
    prediction_node = TrajectoryPrediction()
    rospy.Subscriber(prediction_node.odom_topic, Odometry, prediction_node.odom_callback)
    rospy.Subscriber(prediction_node.accel_topic, AccelWithCovarianceStamped, prediction_node.accel_callback)
    rospy.spin()
