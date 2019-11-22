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
    def __init__(self, visualize=True):
        self.odom_topic = 'odometry/filtered_map'
        self.accel_topic = 'accel/filtered'
        self.viz_dir = 'velocity_errors/'
        self.visualize = visualize
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
        self.delta_l_plot = []
        self.delta_r_plot = []
        print('Booting up node...')
        rospy.init_node('trajectory_prediction', anonymous=True)

        os.makedirs(self.viz_dir, exist_ok=True)

        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

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
            # get prediction with kinematic transition functions used by robot_localization
            x_new = self.x + self.x_vel * t + 0.5 * self.x_accel * t ** 2
            y_new = self.y + self.y_vel * t + 0.5 * self.y_accel * t ** 2
            yaw_new = self.yaw + self.yaw_vel + 0.5 * self.yaw_accel * t ** 2
            x_vel_new = self.x_vel + self.x_accel * t
            y_vel_new = self.y_vel + self.y_accel * t
            yaw_vel_new = self.yaw_vel + self.yaw_accel * t


        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = euler[2]
        self.x_vel = twist.linear.x
        self.y_vel = twist.linear.y
        self.yaw_vel = twist.angular.z
        # compute deviations from actual observations
        x_error = x_new - self.x
        y_error = y_new - self.y
        yaw_error = yaw_new - self.yaw
        x_vel_error = x_vel_new - self.x_vel
        y_vel_error = y_vel_new - self.y_vel
        yaw_vel_error = yaw_vel_new - self.yaw_vel

        delta_l = x_vel_error / math.cos(self.yaw) - (self.yaw_vel * 0.63) / 2
        delta_r = x_vel_error / math.cos(self.yaw) + (self.yaw_vel * 0.63) / 2

        self.delta_l_plot.append(delta_l)
        self.delta_r_plot.append(delta_r)

        print(delta_r, delta_l)
        self.old_timestamp = self.new_timestamp  # set timestamp to be used for prediction as current msg timestamp

        if self.visualize:
            fig = plt.figure(figsize=(16, 8))
            ax = plt.subplot(121)
            ax.set_title('Left side RPM Deviation from Prediction')
            ax.set_ylim(-50, 50)
            ax.plot(self.delta_l_plot, label='x')
            ax.axhline(np.mean(self.delta_l_plot), label='mean', c='r')
            ax.legend(loc='best')

            ax = plt.subplot(122)
            ax.set_title('Right side RPM Deviation from Prediction')
            ax.set_ylim(-50, 50)
            ax.plot(self.delta_r_plot, label='x')
            ax.axhline(np.mean(self.delta_r_plot), label='mean', c='r')
            ax.legend(loc='best')
            fig.savefig(self.viz_dir + 'vel_error_%d.png' % (len(os.listdir(self.viz_dir))))
            plt.close()

    def accel_callback(self, msg):
        linear = msg.accel.accel.linear
        angular = msg.accel.accel.angular
        self.x_accel = linear.x
        self.y_accel = linear.y
        self.yaw_accel = angular.z


if __name__ == '__main__':
    prediction_node = TrajectoryPrediction()
    rospy.Subscriber(prediction_node.odom_topic, Odometry, prediction_node.odom_callback, queue_size=1)
    rospy.Subscriber(prediction_node.accel_topic, AccelWithCovarianceStamped, prediction_node.accel_callback, queue_size=1)
    rospy.spin()
