#!/usr/bin/python2
import rospy
from obstacle_detection.global_occupancy_grid import GlobalOccupancyGrid

if __name__ == '__main__':
    try:
        global_grid = GlobalOccupancyGrid()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
