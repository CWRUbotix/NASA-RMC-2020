#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from canbus.msg import MotorData

topic = 'MotorData'


def position_callback(msg):
    print(msg.can_id)

rospy.init_node('MotorData_listener', anonymous=True)

sub=rospy.Subscriber(topic, MotorData, position_callback)

rospy.spin()
