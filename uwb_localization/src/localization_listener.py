#!/usr/bin/env python3
import os
import sys
import glob
import math
import rospy
import rospkg
import math
import matplotlib
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
from itertools import combinations, permutations
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from canbus.msg import UwbData
from triangulation import UltraWideBandNode
from unscented_localization import run_localization

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point


class LocalizationNode:
    def __init__(self, visualize=True):
        self.topic = 'localization_data'  # topic where UWB distances are published
        self.viz_dir = 'visualizations/'  # directory to store node visualizations
        self.visualize = visualize
        self.viz_step = 50
        print('Booting up node...')
        rospy.init_node('localization_listener', anonymous=True)
        self.nodes = []  # list of UltraWideBandNode instances, one for each node and one for each anchor
        self.robot_x = []  # list of past and current x positions
        self.robot_y = []  # list of past and current y positions
        self.robot_theta = []  # list of past and current yaw measurements
        self.sensors = None  # DataFrame of sensors, types, and relative positions
        self.msg_counts = {}  # dictionary of counts for each node to keep track of number of anchor distances received

        # setup visualization directory and remove all past visualizations
        os.makedirs(self.viz_dir, exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

    def init_nodes(self):
        rp = rospkg.RosPack()
        # find config file in canbus node directory
        script_path = os.path.join(rp.get_path("canbus"), "include", "node_config.csv")
        sensors = pd.read_csv(script_path)
        self.sensors = sensors
        for i in range(len(self.sensors)):
            self.msg_counts[i] = 0  # initialize all message counts to zero
        print(sensors)
        return sensors

    def get_robot_orientation(self):
        non_anchors = [x for x in self.nodes if x.type == 'node']  # get all nodes on the robot
        node_pairs = combinations(non_anchors, 2)  # get all pairwise combinations of nodes
        thetas = []  # list to store bearing angles measured between all pairs
        for (start_node, end_node) in node_pairs:
            if start_node.is_valid() and end_node.is_valid():
                # get vector with tail at start node and head at end end
                robot_edge = -np.array([start_node.relative_x, start_node.relative_y]) + np.array([end_node.relative_x, end_node.relative_y])
                dot_product = np.dot(robot_edge, np.array([1, 0]))
                # each edge vector is at a different angle relative to the robot coordinate system
                theta_offset = np.arctan2(robot_edge[1], robot_edge[0])
                dY = end_node.y - start_node.y
                dX = end_node.x - start_node.x
                theta = np.arctan2(dY, dX) - theta_offset
                thetas.append([np.cos(theta), np.sin(theta)])
        theta = np.mean(thetas, axis=0)
        self.robot_theta.append(np.arctan2(theta[1], theta[0]))
        return self.robot_theta[-1]

    def position_callback(self, msg):
        for node in self.nodes:
            if node.id == msg.node_id:
                node.add_measurement(msg.anchor_id, msg.distance, msg.confidence)
                self.msg_counts[node.id] += 1
        for node in self.nodes:
            if self.msg_counts[node.id] >= 3:
                node.get_position()
                self.msg_counts[node.id] = 0
        avg_x = 0
        avg_y = 0
        total = 0
        for node in self.nodes:
            if node.is_valid():
                avg_x += node.x - node.relative_x
                avg_y += node.y - node.relative_y
                total += 1
        if total > 0:
            self.robot_x.append(avg_x / total)
            self.robot_y.append(avg_y / total)
            theta = self.get_robot_orientation()
            print('X: %.3f, Y: %.3f, theta: %.3f' % (self.robot_x[-1], self.robot_y[-1], theta))
            self.compose_msg()
        if self.visualize and len(self.robot_x) % self.viz_step == 0:
            fig = plt.figure(figsize=(6 * 4, 9))

            ax = plt.subplot(141)
            ax.set_title('Position')
            for node in self.nodes:
                node.plot_position(ax=ax)

            ax.legend(loc='best')
            ax.set_xlim(0, 5.2)
            ax.set_ylim(0, 6.05)

            ax = plt.subplot(142)
            ax.set_title('Node Distances')
            for node in self.nodes:
                ax.plot(node.distance_plot, label=node.id)
            ax.set_ylim(0, 6.05)
            ax.legend(loc='best')

            ax = plt.subplot(143)
            ax.scatter(self.robot_x, self.robot_y, label='robot')
            ax.arrow(self.robot_x[-1], self.robot_y[-1], .3 * math.cos(theta), .3 * math.sin(theta), head_width=0.1)
            ax.legend(loc='best')
            ax.set_xlim(0, 5.2)
            ax.set_ylim(0, 6.05)

            plt.tight_layout()
            fig.savefig(self.viz_dir + 'node_1_%d.png' % (len(os.listdir(self.viz_dir))))
            plt.close()

    def compose_msg(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        point_msg = Point(self.robot_x[-1], self.robot_y[-1], 0)  # use most recent pos with no z-coord
        orientation_quat = R.from_euler('xyz', [0, 0, self.robot_theta[-1]]).as_quat()  # pitch is rotation about z-axis in euler angles
        pose_cov = np.diag([0.01, 0.01, 0, 0, 0, 0.04]).flatten()
        quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = Pose(point_msg, quat_msg)
        pose_with_cov.covariance = pose_cov
        stamped_msg = PoseWithCovarianceStamped()
        stamped_msg.header = header
        stamped_msg.pose = pose_with_cov
        try:
            pub = rospy.Publisher('uwb_nodes', PoseWithCovarianceStamped, queue_size=1)
            #rospy.loginfo(stamped_msg)
            pub.publish(stamped_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass


if __name__ == '__main__':
    localization_node = LocalizationNode()
    sensors = localization_node.init_nodes()
    for i, sensor in sensors.iterrows():
        if sensor['type'] == 'node':
            uwb_node = UltraWideBandNode(sensor['id'], sensor['x'], sensor['y'], sensor['type'], sensors)
            localization_node.nodes.append(uwb_node)

    sub = rospy.Subscriber(localization_node.topic, UwbData, localization_node.position_callback, queue_size=12)
    rospy.spin()
