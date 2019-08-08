#!/usr/bin/env python3
import os
import rospy
import rospkg
import math
import matplotlib
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import matplotlib.pyplot as plt
from canbus.msg import UWB_data
from triangulation import UltraWideBandNode

topic = 'localization_data'
viz_dir = 'visualizations/'
print('Booting up node...')
rospy.init_node('localization_listener', anonymous=True)
distances = {}
confidences = {}
Nodes = []


def init_nodes():
    rp = rospkg.RosPack()
    script_path = os.path.join(rp.get_path("canbus"), "include", "node_config.csv")
    sensors = pd.read_csv(script_path, index=False)
    print(sensors)


def node_anchor_pair(n_id, a_id):
    return str(n_id) + ', ' + str(a_id)

def position_callback(msg):
    print('distance:', msg.distance, 'confidence:', msg.confidence)
    key_pair = node_anchor_pair(msg.node_id, msg.anchor_id)
    if key_pair in distances.keys():
        distances[key_pair].append(msg.distance)
    else:
        distances[key_pair] = [msg.distance]
    if key_pair in confidences.keys():
        confidences[key_pair].append(msg.confidence)
    else:
        confidences[key_pair] = [msg.confidence]
    ax = plt.subplot(211)
    for key in distances.keys():
        ax.plot(distances[key], label=key)
    ax.set_title('Distance')
    ax.set_ylim(0, 6)
    ax.legend(loc='best')
    ax = plt.subplot(212)
    for key in confidences.keys():
        ax.plot(confidences[keys], label=key)
    ax.set_title('Confidence')
    ax.legend(loc='best')
    plt.savefig('node_1_%d.png' % (len(os.listdir('.'))))
    plt.close()

if __name__ == '__main__':
    init_nodes()
    sub=rospy.Subscriber(topic, UWB_data, position_callback)
    rospy.spin()
