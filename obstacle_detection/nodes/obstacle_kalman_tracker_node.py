#!/usr/bin/env python2
import rospy
from obstacle_detection.obstacle_kalman_tracker import ObstacleKalmanTracker

if __name__ == '__main__':
    try:
        obstacle_kalman_tracker = ObstacleKalmanTracker()
    except rospy.exceptions.ROSInterruptException:
        pass
