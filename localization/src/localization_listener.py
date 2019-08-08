#!/usr/bin/env python
import os
import rospy
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from localization.msg import UWB_data

topic = 'localization_data'
viz_dir = 'visualizations/'
rospy.init_node('localization_listener', anonymous=True)
distances = []
confidences = []

def position_callback(msg):
    print('distance:', msg.distance, 'confidence:', msg.confidence)
    if msg.node_id == 1 and msg.anchor_id == 17:
        distances.append(msg.distance)
        confidences.append(msg.confidence)
    ax = plt.subplot(211)
    ax.plot(distances)
    ax.set_title('Distance')
    ax.set_ylim(0, 6)
    ax = plt.subplot(212)
    ax.plot(confidences)
    ax.set_title('Confidence')
    plt.savefig('node_1_%d.png' % (len(os.listdir('.'))))
    plt.close()

sub=rospy.Subscriber(topic, UWB_data, position_callback)
rospy.spin()
