#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from obstacle_detection.msg import Path

topic = 'path_data'
#node_name = 'automodule'


def send_path_data(path):
    """
    Publishes the ID, XYZ coordinates, and diameter in a custom Obstacle message.  All measurements are in meters

    Args:
        obs (:obj:`Obstacle`) : obstacle object containing the ID, coordinates, and diameter of the obstacle to publish

    """
    global topic
    point_list = []
    for pos in path.path:
        point = Point()
        point.x = pos.getX()
        point.y = pos.getY()
        point_list.append(point)
		
    try:
        pub = rospy.Publisher(topic, Path, queue_size=10)
        #rospy.init_node(node_name)
        print(topic)
        msg = Path()
        msg.points = point_list
        rospy.loginfo(msg)
        pub.publish(msg)
    except rospy.ROSInterruptException as e:
        print(e.getMessage())
        pass
