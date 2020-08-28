#!/usr/bin/python3
import rospy
from obstacle_detection.global_occupancy_grid import GlobalOccupancyGrid

if __name__ == '__main__':
    try:
        global_grid = GlobalOccupancyGrid()
        rospy.Subscriber(global_grid.local_grid_topic, OccupancyGrid, global_grid.local_grid_callback, queue_size=1)
        rospy.Subscriber(global_grid.localization_topic, Odometry, global_grid.localization_listener, queue_size=1)
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
