#!/usr/bin/env python3
import os
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
from canbus.msg import UWB_data
from triangulation import UltraWideBandNode


class LocalizationNode:
    def __init__(self):
        self.topic = 'localization_data'
        self.viz_dir = 'visualizations/'
        print('Booting up node...')
        rospy.init_node('localization_listener', anonymous=True)
        self.nodes = []
        self.robot_x = []
        self.robot_y = []
        self.robot_theta = []
        self.sensors = None
        self.msg_counts = {1:0, 2:0, 3:0}

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
        print(sensors)
        return sensors

    def get_robot_orientation(self):
        non_anchors = [x for x in self.nodes if x.type == 'node']
        node_pairs = combinations(non_anchors, 2)
        thetas = []
        for (node_1, node_2) in node_pairs:
            #if node_1.relative_x < 0 and node_1.relative_y < 0:
            if node_1.relative_x == node_2.relative_x and node_1.relative_y > node_2.relative_y:
                node_1, node_2 = node_2, node_1  # swap nodes
            elif node_1.relative_y == node_2.relative_y and node_1.relative_x > node_2.relative_x:
                node_1, node_2 = node_2, node_1
            elif node_1.relative_y > node_2.relative_y or node_1.relative_x > node_2.relative_x:
                node_1, node_2 = node_2, node_1
            robot_edge = -np.array([node_1.relative_x, node_1.relative_y]) + np.array([node_2.relative_x, node_2.relative_y])
            dot_product = np.dot(robot_edge, np.array([1, 0]))
            theta_offset = math.acos(dot_product / np.linalg.norm(robot_edge))
            print(node_1.id, node_2.id, theta_offset)
            dY = node_2.y - node_1.y
            dX = node_2.x - node_1.x
            theta = math.atan2(dY, dX) - theta_offset
            thetas.append(theta)
        print(thetas)
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
        theta = self.get_robot_orientation()
        print(theta)

        #self.remove_invalid_points()

        fig = plt.figure(figsize=(6 * 3, 9))
        ax = plt.subplot(131)
        ax.set_title('Position')
        for node in self.nodes:
            node.plot_position(ax=ax)
        #ax.axis('equal')
        ax.legend(loc='best')
        ax.set_xlim(0, 4.2)
        ax.set_ylim(0, 6.05)
        ax = plt.subplot(132)
        ax.set_title('Node Distances')
        for node in self.nodes:
            ax.plot(node.distance_plot, label=node.id)
        ax.set_ylim(0, 6.05)
        ax.legend(loc='best')
        ax = plt.subplot(133)
        avg_x = 0
        avg_y = 0
        total = 0
        for node in self.nodes:
            if node.x is not None and node.y is not None:
                avg_x += node.x
                avg_y += node.y
                total += 1
        self.robot_x.append(avg_x / total)
        self.robot_y.append(avg_y / total)
        arrow_x = .3 * math.cos(theta)
        arrow_y = .3 * math.sin(theta)
        ax.scatter(self.robot_x, self.robot_y, label='robot')
        ax.arrow(self.robot_x[-1], self.robot_y[-1], arrow_x, arrow_y, head_width=0.1)
        ax.legend(loc='best')
        ax.set_xlim(0, 4.2)
        ax.set_ylim(0, 6.05)
        plt.tight_layout()
        fig.savefig(self.viz_dir + 'node_1_%d.png' % (len(os.listdir(self.viz_dir))))
        plt.close()


if __name__ == '__main__':
    localization_node = LocalizationNode()
    sensors = localization_node.init_nodes()
    for i, sensor in sensors.iterrows():
        if sensor['type'] == 'node':
            uwb_node = UltraWideBandNode(sensor['id'], sensor['x'], sensor['y'], sensor['type'], sensors)
            localization_node.nodes.append(uwb_node)

    sub = rospy.Subscriber(localization_node.topic, UWB_data, localization_node.position_callback, queue_size=18)
    rospy.spin()
