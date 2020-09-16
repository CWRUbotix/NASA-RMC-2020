#!/usr/bin/python3
import rospy
from glenn_utilities.ros_bagger import RosBagger

if __name__ == '__main__':
    try:
        ros_bagger = RosBagger()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
