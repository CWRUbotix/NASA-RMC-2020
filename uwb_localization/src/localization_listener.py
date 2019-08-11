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
    global nodes
    for node in nodes:
        if node.id == msg.node_id:
            node.add_measurement(msg.anchor_id, msg.distance, msg.confidence)
    for node in nodes:
        node.get_position()

    remove_invalid_points(nodes)

    fig = plt.figure(figsize=(16, 8))
    ax = plt.subplot(121)
    ax.set_title('Position')
    for node in nodes:
        node.plot_position(ax=ax)
    ax.axis('equal')
    '''
    x_pos = np.array([])
    y_pos = np.array([])
    for node in nodes:
        if len(node.x_plot) == len(node.y_plot):
            x_pos = np.append(x_pos, node.x_plot)
            y_pos = np.append(y_pos, node.y_plot)
    x_pos = np.reshape(x_pos, (len(nodes), -1))
    y_pos = np.reshape(y_pos, (len(nodes), -1))
    print(x_pos.shape, y_pos.shape)
    ax.scatter(np.mean(x_pos, axis=0), np.mean(y_pos, axis=0), label='robot')
    '''
    ax.legend(loc='best')
    ax = plt.subplot(122)
    ax.set_title('Position (moving average)')
    for node in nodes:
        node.plot_position(ax=ax, moving_average=True)
    ax.axis('equal')
    ax.legend(loc='best')
    plt.tight_layout()
    fig.savefig(viz_dir + 'node_1_%d.png' % (len(os.listdir(viz_dir))))
    plt.close()


if __name__ == '__main__':
    sensors = init_nodes()
    for i, sensor in sensors.iterrows():
        if sensor['type'] == 'node':
            nodes.append(UltraWideBandNode(sensor['id'], sensors))
    sub = rospy.Subscriber(topic, UWB_data, position_callback)
    rospy.spin()
