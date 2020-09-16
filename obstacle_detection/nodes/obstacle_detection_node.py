#!/usr/bin/python3
import rospy
from obstacle_detection.obstacle_detection import ObstacleDetectionNode

if __name__ == '__main__':
    try:
        obstacle_detection = ObstacleDetectionNode()
    except rospy.exceptions.ROSInterruptException:
        pass
