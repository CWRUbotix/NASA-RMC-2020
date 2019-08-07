#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32

topic = 'localization_data'


def position_callback(msg):
    print(msg.data)

rospy.init_node('localization_listener', anonymous=True)

sub=rospy.Subscriber(topic, Float32, position_callback)

rospy.spin()
