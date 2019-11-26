#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from canbus.msg import VescData

topic = 'VescData'


def position_callback(msg):
    print(msg.can_id)

rospy.init_node('VescData_listener', anonymous=True)

sub=rospy.Subscriber(topic, VescData, position_callback)

rospy.spin()
