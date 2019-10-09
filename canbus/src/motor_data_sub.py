#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from canbus.msg import motor_data

topic = 'motor_data'


def position_callback(msg):
    print(msg.can_id)

rospy.init_node('motor_data_listener', anonymous=True)

sub=rospy.Subscriber(topic, motor_data, position_callback)

rospy.spin()
