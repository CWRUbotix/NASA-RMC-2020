#!/usr/bin/env python2
import numpy as np
import math

import rospy
import tf2_ros, tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

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

        self.num_measurements = 1  # How many times this obstacle has been seen
        self.num_missed = 0  # How many times it wasn't seen but could have been

        self.seen = False  # If this obstacle was measured this timestep

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
        # print(obj.x[:2])
        # print(self.x[:2])
        # print(self.P[:2])
        # print()

    def wrap_angle(self, x):
        return (x + 3.1416) % 6.283 - 3.1416

    def could_have_been_seen(self, camera):
        dx = self.x[0] - camera.translation.x
        dy = self.x[1] - camera.translation.y
        distance = dx**2 + dy**2

        # If too close, we can't see it
        if distance < 0.09:  # 30 cm squared
            return False

        angle = math.atan2(dy, dx)

        _, _, yaw = euler_from_quaternion([camera.rotation.x, camera.rotation.y, camera.rotation.z, camera.rotation.w])

        # Use modulus to check if the angle difference from 90 degrees is within field of view
        # Camera forward is 90 degrees from where 0 degrees is
        # This doesn't account for the obstacles being spherical not points
        return abs(self.wrap_angle(angle - yaw - 1.57)) < 0.74  # 85 degree fov

    def display(self):
        print("me: " + str(self.num_measurements) + ", mi: " + str(self.num_missed))

    def get_measured_ratio(self):
        return float(self.num_measurements) / (self.num_measurements + self.num_missed)

    def likely_exists(self):
        return (self.num_measurements + self.num_missed) >= 6 and self.get_measured_ratio() > 0.7

    def likely_doesnt_exist(self):
        return (self.num_measurements + self.num_missed) >= 6 and self.get_measured_ratio() <= 0.4


# Tracks all the obstacles
class ObstacleKalmanTracker:
    def __init__(self):
        rospy.init_node("obstacle_kalman_tracker")
        rospy.loginfo("Obstacle tracker node initialized")

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        self.obstacles = []  # TrackedObstacle array

        rospy.Subscriber('input_models', ModelCoefficientsArray, self.receive_model_coefficients, queue_size=1)
        self.model_viz_pub = rospy.Publisher('output_models', Marker, queue_size=10)
        self.model_pub = rospy.Publisher('output_models_test', PointStamped, queue_size=5)

        self.numpy_data = np.empty((0, 4))

        # rospy.on_shutdown(self.on_shutdown)
        rospy.spin()

    def on_shutdown(self):
        # np.save("/home/edf42001/data.npy", self.numpy_data)
        pass

    def receive_model_coefficients(self, msg):
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

            # self.numpy_data = np.vstack((self.numpy_data, [[center.point.x, center.point.y, center.point.z, msg.values[i+3]]]))

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
                    obstacle.num_measurements += 1
                    obstacle.seen = True
                    new_obs = False
                    break

            if new_obs:
                new_obstacles.append(potential_object)

        # Add new potential obstacles
        self.obstacles += new_obstacles

        try:
            camera_location = self.tf_buffer.lookup_transform("map", "realsense_footprint_link", msg.header.stamp, rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn(e)
            return

        for obs in self.obstacles:
            # obs.display()
            # print(obs.get_measured_ratio())

            if not obs.seen and obs.could_have_been_seen(camera_location.transform):
                obs.num_missed += 1
            else:
                obs.seen = False  # Reset seen for use next timestep

        for i in range(len(self.obstacles)-1, 0 ,-1):
            if self.obstacles[i].likely_doesnt_exist():
                del self.obstacles[i]
        # print()

        self.publish_model_markers(msg_obstacles, msg.header.stamp, "potential_obstacles", 0.3, camera_location.transform)
        self.publish_model_markers(self.obstacles, msg.header.stamp, "tracked_obstacles", 1.0, camera_location.transform)

        # print(len(self.obstacles))

    def publish_model_markers(self, models, stamp, ns, a, cam):
        # Clear visualization
        self.model_viz_pub.publish(Marker(ns=ns, action=Marker.DELETEALL))

        # Publish all the viz
        for i in range(len(models)):
            seen = models[i].could_have_been_seen(cam)
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

            marker.color.r = (0 if seen else 1) * models[i].get_measured_ratio()
            marker.color.g = (1 if seen else 0) * models[i].get_measured_ratio()
            marker.color.b = 0.0
            marker.color.a = a if models[i].likely_exists() else 0.3

            # Publish the marker
            self.model_viz_pub.publish(marker)
