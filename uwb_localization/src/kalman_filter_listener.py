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
from filterpy.stats import plot_covariance
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry



class KalmanFilterNode:
    def __init__(self):
        self.topic = 'odometry/filtered_map'
        self.accel_topic = 'accel/filtered'
        print('Booting up node...')
        rospy.init_node('kalman_filter_listener', anonymous=True)
        self.robot_x = []
        self.robot_y = []
        self.unfiltered_x = []
        self.unfiltered_y = []
        self.unfiltered_yaw = []
        self.node_x = []
        self.node_y = []
        self.robot_yaw = []
        self.node_yaw = []
        self.viz_dir = 'robot_localization_viz'
        self.viz_step = 50

        os.makedirs(self.viz_dir, exist_ok=True)

        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

    def confidence_ellipse(self, x, y, cov, ax, n_std=3.0, facecolor='none', **kwargs):
        """
        Create a plot of the covariance confidence ellipse of *x* and *y*.

        Parameters
        ----------
        x, y : array-like, shape (n, )
            Input data.

        ax : matplotlib.axes.Axes
            The axes object to draw the ellipse into.

        n_std : float
            The number of standard deviations to determine the ellipse's radiuses.

        Returns
        -------
        matplotlib.patches.Ellipse

        Other parameters
        ----------------
        kwargs : `~matplotlib.patches.Patch` properties
        """

        pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
        # Using a special case to obtain the eigenvalues of this
        # two-dimensionl dataset.
        ell_radius_x = np.sqrt(1 + pearson)
        ell_radius_y = np.sqrt(1 - pearson)
        ellipse = Ellipse((0, 0),
            width=ell_radius_x * 2,
            height=ell_radius_y * 2,
            facecolor=facecolor,
            **kwargs)

        # Calculating the stdandard deviation of x from
        # the squareroot of the variance and multiplying
        # with the given number of standard deviations.
        scale_x = np.sqrt(cov[0, 0]) * n_std
        mean_x = x

        # calculating the stdandard deviation of y ...
        scale_y = np.sqrt(cov[1, 1]) * n_std
        mean_y = y

        transf = transforms.Affine2D() \
            .rotate_deg(45) \
            .scale(scale_x, scale_y) \
            .translate(mean_x, mean_y)

        ellipse.set_transform(transf + ax.transData)
        return ax.add_patch(ellipse)

    def unfiltered_callback(self, msg):
        pose = msg.pose.pose
        self.unfiltered_x.append(pose.position.x)
        self.unfiltered_y.append(pose.position.y)
        quat = pose.orientation
        euler = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
        self.unfiltered_yaw.append(euler[2])  # rotation about vertical z-axis
        #print('Var(X): %0.6f' % np.var(self.unfiltered_x), 'Var(Y): %0.6f' % np.var(self.unfiltered_y))

    def position_callback(self, msg):
        pose = msg.pose.pose
        covariance = msg.pose.covariance
        covariance = np.reshape(covariance, (6, 6))

        # print(["{:0.1f}".format(i) for i in msg.pose.covariance])
        # print(["{:0.1f}".format(i) for i in msg.twist.covariance])
        #print(msg.pose.covariance)
        #print(msg.twist.covariance)


        self.robot_x.append(pose.position.x)
        self.robot_y.append(pose.position.y)
        quat = pose.orientation
        euler = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
        self.robot_yaw.append(euler[2])  # rotation about vertical z-axis
        print('X: %.4f \tY: %.4f \tyaw: %.4f' % (self.robot_x[-1], self.robot_y[-1], self.robot_yaw[-1]))

        if len(self.robot_x) % self.viz_step == 0:
            fig, ax = plt.subplots(figsize=(6, 9))
            if len(self.unfiltered_x) > 0 and len(self.unfiltered_y) > 0 and len(self.unfiltered_x) == len(self.unfiltered_y):
                ax.scatter(self.unfiltered_x, self.unfiltered_y, label='raw UWB', alpha=0.2, marker='+')
            ax.plot(self.robot_x, self.robot_y, label='kalman filter', alpha=0.75, c='tab:orange')
            self.confidence_ellipse(self.robot_x[-1], self.robot_y[-1], covariance[0: 2, 0: 2], ax, edgecolor='red')
            ax.arrow(self.robot_x[-1], self.robot_y[-1], .3 * math.cos(self.robot_yaw[-1]), .3 * math.sin(self.robot_yaw[-1]), head_width=0.1)
            ax.set_xlim(0, 5.2)
            ax.set_ylim(0, 6.05)
            ax.legend(loc='best')
            plt.tight_layout()
            fig.savefig(self.viz_dir + '/localization_%d.png' % (len(os.listdir(self.viz_dir))))
            plt.close()

    def node_callback(self, msg):
        self.node_x.append(msg.pose.pose.position.x)
        self.node_y.append(msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        euler = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
        self.node_yaw.append(euler[2])

    def accel_callback(self, msg):
        linear = msg.accel.accel.linear
        angular = msg.accel.accel.angular
        #print(["{:0.3f}".format(i) for i in msg.accel.covariance])


if __name__ == '__main__':
    kalman_filter_node = KalmanFilterNode()
    rospy.Subscriber(kalman_filter_node.topic, Odometry, kalman_filter_node.position_callback, queue_size=1)
    rospy.Subscriber(kalman_filter_node.accel_topic, AccelWithCovarianceStamped, kalman_filter_node.accel_callback, queue_size=1)
    rospy.Subscriber('uwb_nodes', PoseWithCovarianceStamped, kalman_filter_node.unfiltered_callback, queue_size=1)

    rospy.spin()
