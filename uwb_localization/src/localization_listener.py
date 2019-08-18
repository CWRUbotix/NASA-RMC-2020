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
from canbus.msg import UWB_data
from triangulation import UltraWideBandNode

topic = 'localization_data'
viz_dir = 'visualizations/'
print('Booting up node...')
rospy.init_node('localization_listener', anonymous=True)
nodes = []
robot_x = []
robot_y = []
msg_counts = {1:0, 2:0, 3:0}

try:
    files = glob.glob('%s/*' % viz_dir)
    for f in files:
        os.remove(f)
except Exception as e:
    print(e)


def init_nodes():
    rp = rospkg.RosPack()
    script_path = os.path.join(rp.get_path("canbus"), "include", "node_config.csv")
    sensors = pd.read_csv(script_path)
    print(sensors)
    return sensors


def euclidean_distance(x1, x2, y1, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def remove_invalid_points(nodes, epsilon=.2):
    best_node = nodes[0]
    for node in nodes:
        if node.confidence > best_node.confidence:
            best_node = node
    for n1 in nodes:
        if n1.id != best_node.id:  # compare to the highest confidence measure
            for n2 in nodes:
                if n1.id != n2.id and n2.id == best_node.id:  # if nodes are not the same
                    measured_distance = euclidean_distance(n1.x, n2.x, n1.y, n2.y)
                    robot_x1, robot_y1 = n1.get_robot_position()
                    robot_x2, robot_y2 = n2.get_robot_position()
                    robot_distance = euclidean_distance(robot_x1, robot_x2, robot_y1, robot_y2)
                    if abs(measured_distance - robot_distance) > epsilon:
                        n1.x = n1.x_plot[-2]  # set position to most recent valid measure
                        n1.y = n1.y_plot[-2]
                        n1.x_plot = n1.x_plot[:-1]
                        n1.y_plot = n1.y_plot[:-1]
                        break


def position_callback(msg):
    global nodes, robot_x, robot_y, msg_counts
    for node in nodes:
        if node.id == msg.node_id:
            node.add_measurement(msg.anchor_id, msg.distance, msg.confidence)
            msg_counts[node.id] += 1
    for node in nodes:
        if msg_counts[node.id] >= 3:
            node.get_position()

    #remove_invalid_points(nodes)

    fig = plt.figure(figsize=(6 * 3, 9))
    ax = plt.subplot(131)
    ax.set_title('Position')
    for node in nodes:
        node.plot_position(ax=ax)
    #ax.axis('equal')
    ax.legend(loc='best')
    ax.set_xlim(0, 4.2)
    ax.set_ylim(0, 6.05)
    ax = plt.subplot(132)
    ax.set_title('Node Distances')
    for node in nodes:
        ax.plot(node.distance_plot, label=node.id)
    ax.set_ylim(0, 6.05)
    ax.legend(loc='best')
    ax = plt.subplot(133)
    avg_x = 0
    avg_y = 0
    total = 0
    for node in nodes:
        if node.x is not None and node.y is not None:
            avg_x += node.x
            avg_y += node.y
            total += 1
    robot_x.append(avg_x / total)
    robot_y.append(avg_y / total)
    ax.scatter(robot_x, robot_y, label='robot')
    #ax.axis('equal')
    ax.legend(loc='best')
    ax.set_xlim(0, 4.2)
    ax.set_ylim(0, 6.05)
    plt.tight_layout()
    fig.savefig(viz_dir + 'node_1_%d.png' % (len(os.listdir(viz_dir))))
    plt.close()


if __name__ == '__main__':
    sensors = init_nodes()
    for i, sensor in sensors.iterrows():
        if sensor['type'] == 'node':
            nodes.append(UltraWideBandNode(sensor['id'], sensors))
    sub = rospy.Subscriber(topic, UWB_data, position_callback, queue_size=18)
    rospy.spin()
