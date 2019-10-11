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
from canbus.msg import UWB_data
from triangulation import UltraWideBandNode
from unscented_localization import run_localization

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point


class LocalizationNode:
    def __init__(self, visualize=True):
        self.topic = 'localization_data'
        self.viz_dir = 'visualizations/'
        self.visualize = visualize
        print('Booting up node...')
        rospy.init_node('localization_listener', anonymous=True)
        self.nodes = []
        self.robot_x = []
        self.robot_y = []
        self.kalman_x = []
        self.kalman_y = []
        self.robot_theta = []
        self.kalman_theta = []
        self.sensors = None
        self.msg_counts = {}

        os.makedirs(self.viz_dir, exist_ok=True)

        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

    def init_nodes(self):
        rp = rospkg.RosPack()
        script_path = os.path.join(rp.get_path("canbus"), "include", "node_config.csv")
        sensors = pd.read_csv(script_path)
        self.sensors = sensors
        for i in range(len(self.sensors)):
            self.msg_counts[i] = 0
        print(sensors)
        return sensors

    def get_robot_orientation(self):
        non_anchors = [x for x in self.nodes if x.type == 'node']  # get all nodes on the robot
        node_pairs = combinations(non_anchors, 2)  # get all pairwise combinations of nodes
        thetas = []  # list to store bearing angles measured between all pairs
        for (start_node, end_node) in node_pairs:
            if start_node.is_valid() and end_node.is_valid():
                # horizontally parallel but wrong direction
                if start_node.relative_x == end_node.relative_x and start_node.relative_y > end_node.relative_y:
                    start_node, end_node = end_node, start_node  # swap nodes
                # vertically parallel but wrong direction
                elif start_node.relative_y == end_node.relative_y and start_node.relative_x > end_node.relative_x:
                    start_node, end_node = end_node, start_node
                # diagonal and wrong direction
                elif start_node.relative_y > end_node.relative_y or start_node.relative_x > end_node.relative_x:
                    start_node, end_node = end_node, start_node
                # get vector with tail at start node and head at end end
                robot_edge = -np.array([start_node.relative_x, start_node.relative_y]) + np.array([end_node.relative_x, end_node.relative_y])
                dot_product = np.dot(robot_edge, np.array([1, 0]))
                # each edge vector is at a different angle relative to the robot coordinate system
                theta_offset = math.acos(dot_product / np.linalg.norm(robot_edge))
                dY = end_node.y - start_node.y
                dX = end_node.x - start_node.x
                theta = math.atan2(dY, dX) - theta_offset
                thetas.append(theta)
        theta = np.mean(thetas)
        self.robot_theta.append(theta)
        return theta

    @staticmethod
    def euclidean_distance(x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def remove_invalid_points(self, epsilon=.2):
        best_node = self.nodes[0]
        for node in self.nodes:
            if node.confidence > best_node.confidence:
                best_node = node
        for n1 in self.nodes:
            if n1.id != best_node.id:  # compare to the highest confidence measure
                for n2 in self.nodes:
                    if n1.id != n2.id and n2.id == best_node.id:  # if nodes are not the same
                        measured_distance = self.euclidean_distance(n1.x, n2.x, n1.y, n2.y)
                        robot_x1, robot_y1 = n1.get_robot_position()
                        robot_x2, robot_y2 = n2.get_robot_position()
                        robot_distance = self.euclidean_distance(robot_x1, robot_x2, robot_y1, robot_y2)
                        if abs(measured_distance - robot_distance) > epsilon:
                            n1.x = n1.x_plot[-2]  # set position to most recent valid measure
                            n1.y = n1.y_plot[-2]
                            n1.x_plot = n1.x_plot[:-1]
                            n1.y_plot = n1.y_plot[:-1]
                            break

    def position_callback(self, msg):
        for node in self.nodes:
            if node.id == msg.node_id:
                node.add_measurement(msg.anchor_id, msg.distance, msg.confidence)
                self.msg_counts[node.id] += 1
        for node in self.nodes:
            if self.msg_counts[node.id] >= 3:
                node.get_position()
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
            self.compose_msg()
        if self.visualize:
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
        for node in self.nodes:
            if node.is_valid():
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'map'
                point_msg = Point(node.x - node.relative_x, node.y - node.relative_y, 0)  # use most recent pos with no z-coord
                orientation_quat = R.from_euler('xyz', [0, 0, self.robot_theta[-1]]).as_quat()  # pitch is rotation about z-axis in euler angles
                pose_cov = np.ones(36) * 1e-9
                quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
                pose_with_cov = PoseWithCovariance()
                pose_with_cov.pose = Pose(point_msg, quat_msg)
                pose_with_cov.covariance = pose_cov
                stamped_msg = PoseWithCovarianceStamped()
                stamped_msg.header = header
                stamped_msg.pose = pose_with_cov
                try:
                    pub = rospy.Publisher('uwb_node_%d' % node.id, PoseWithCovarianceStamped, queue_size=10)
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

    sub = rospy.Subscriber(localization_node.topic, UWB_data, localization_node.position_callback)
    rospy.spin()
