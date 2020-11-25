#!/usr/bin/env python2
import numpy as np

import rospy
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import Pose, Vector3, Point, PointStamped
from visualization_msgs.msg import Marker

from glenn_msgs.msg import ModelCoefficientsArray


# An obstacle that is being tracked
class TrackedObstacle:
    def __init__(self, x):
        # Assume all variables independent for now
        self.x = np.array(x)  # x, y, z, r
        self.P = np.array([0.0015, 0.0015, 0.0015, 0.0015])  # Current state covariance
        self.R = np.array([0.0015, 0.0015, 0.0015, 0.0015])
        self.Q = np.array([1E-5, 1E-5, 1E-5, 1E-5])

        self.num_measurements = 0  # How many times this obstacle has been seen

    def is_probably_the_same(self, obj):
        sigmas = (obj.x - self.x) ** 2 / self.R
        total = np.sqrt(np.sum(sigmas))

        if total < 6:
            # print("%.2f, %.2f" % (obj.x[0], obj.x[1]))
            return True

        return False

    def merge_with(self, obj):        
        self.x = (self.x * self.R + obj.x * self.P) / (self.R + self.P)
        self.P = (self.R * self.P) / (self.R + self.P)

        self.P += self.Q

        self.num_measurements += 1

        # print(obj.x[:2])
        # print(self.x[:2])
        # print(self.P[:2])
        # print()



# Tracks all the obstacles
class ObstacleKalmanTracker:
    def __init__(self):
        rospy.init_node("obstacle_kalman_tracker")
        rospy.loginfo("Obstacle tracker node initialized")

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        self.obstacles = []  # TrackedObstacle array
        self.obstacles.append(TrackedObstacle([1.85, 2.1, 0.0, 0.2]))

        rospy.Subscriber('input_models', ModelCoefficientsArray, self.receive_model_coefficients, queue_size=1)
        self.model_viz_pub = rospy.Publisher('output_models', Marker, queue_size=10)
        self.model_pub = rospy.Publisher('output_models_test', PointStamped, queue_size=5)

        rospy.spin()

    def receive_model_coefficients(self, msg):
        ns = "tracked_obstacles"
        msg_obstacles = []

        # Convert models datatype
        for i in range(0, len(msg.values), msg.step):
            # Copy data into a point
            center = PointStamped(header=msg.header, point=Point(msg.values[i], msg.values[i+1], msg.values[i+2]))

            # Transform to map frame
            try:
                center = self.tf_buffer.transform(center, "map", rospy.Duration(0.1))
            except Exception as e:
                rospy.logwarn(e)
                return

            msg_obstacles.append(TrackedObstacle([center.point.x, center.point.y, center.point.z, msg.values[i+3]]))

            self.model_pub.publish(center)  # Just to test
        new_obstacles = []
        # Check each new object
        for potential_object in msg_obstacles:
            new_obs = True
            # See if it is a measurement of an existing object
            for obstacle in self.obstacles:
                if obstacle.is_probably_the_same(potential_object):
                    obstacle.merge_with(potential_object)
                    new_obs = False
                    break

            if new_obs:
                new_obstacles.append(potential_object)

        self.obstacles += new_obstacles

        self.publish_model_markers(msg_obstacles, msg.header.stamp, "potential_obstacles", 0.5)
        self.publish_model_markers(self.obstacles, msg.header.stamp, "tracked_obstacles", 1.0)

        print(len(self.obstacles))

    def publish_model_markers(self, models, stamp, ns, a):
        # Clear visualization
        self.model_viz_pub.publish(Marker(ns=ns, action=Marker.DELETEALL))

        # Publish all the viz
        for i in range(len(models)):
            # Create visualization
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "map"

            marker.ns = ns  # Set namespace
            marker.id = i  # Give unique id

            marker.type = Marker.SPHERE

            marker.pose = Pose(position=Point(models[i].x[0], models[i].x[1], models[i].x[2]))
            marker.pose.orientation.w = 1.0

            diameter = 2 * models[i].x[3]
            marker.scale = Vector3(diameter, diameter, diameter)

            marker.color.r = models[i].num_measurements * 0.04
            marker.color.g = 0.0
            marker.color.b = models[i].num_measurements * 0.04
            marker.color.a = a

            # Publish the marker
            self.model_viz_pub.publish(marker)
