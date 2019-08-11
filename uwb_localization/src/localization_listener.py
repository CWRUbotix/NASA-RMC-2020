#!/usr/bin/env python3
import os
import rospy
import rospkg
import math
import matplotlib
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import pandas as pd
import matplotlib.pyplot as plt
from canbus.msg import UWB_data
from triangulation import UltraWideBandNode

topic = 'localization_data'
viz_dir = 'visualizations/'
print('Booting up node...')
rospy.init_node('localization_listener', anonymous=True)
nodes = []


def init_nodes():
    rp = rospkg.RosPack()
    script_path = os.path.join(rp.get_path("canbus"), "include", "node_config.csv")
    sensors = pd.read_csv(script_path)
    print(sensors)
    return sensors


def node_anchor_pair(n_id, a_id):
    return str(n_id) + ', ' + str(a_id)


def position_callback(msg):
    global nodes
    key_pair = node_anchor_pair(msg.node_id, msg.anchor_id)
    for node in nodes:
        if node.id == msg.node_id:
            node.add_measurement(msg.anchor_id, msg.distance)
    for node in nodes:
        node.get_position()
    ax = plt.subplot(121)
    ax.set_title('Position')
    for node in nodes:
        node.plot_position(ax=ax)
    ax.legend(loc='best')
    ax = plt.subplot(122)
    ax.set_title('Position (moving average)')
    for node in nodes:
        node.plot_position(ax=ax, moving_average=True)
    ax.legend(loc='best')
    plt.savefig(viz_dir + 'node_1_%d.png' % (len(os.listdir(viz_dir))))
    plt.close()


if __name__ == '__main__':
    sensors = init_nodes()
    for i, sensor in sensors.iterrows():
        if sensor['type'] == 'node':
            nodes.append(UltraWideBandNode(sensor['id'], sensors))
    sub = rospy.Subscriber(topic, UWB_data, position_callback)
    rospy.spin()
