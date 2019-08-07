#!/usr/bin/env python

import rospy
import math
from localization.msg import UWB_data

topic = 'localization_data'
rospy.init_node('localization_listener', anonymous=True)

def position_callback(msg):
    print('distance:', msg.distance, 'confidence:', msg.confidence)


sub=rospy.Subscriber(topic, UWB_data, position_callback)
rospy.spin()
