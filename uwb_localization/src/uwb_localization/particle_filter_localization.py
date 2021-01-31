#!/usr/bin/env python3
import numpy as np

import rospy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, Vector3
from visualization_msgs.msg import Marker

from glenn_msgs.msg import ModelCoefficientsArray
from uwb_localization.particle_filter.particle_filter import ParticleFilter


class ParticleFilterLocalization:
    def __init__(self):
        rospy.init_node("partice_filter_localization")
        rospy.loginfo("Particle filter node initialized")

        # XY coordinates of known obstacles
        self.world = np.array([[0, 1.5]])

        self.n_particles = 1000
        self.vel_noise_sigma = 0.04
        self.angular_vel_noise_sigma = 0.04

        self.fov = 1.42
        self.max_dist = 3

        self.particles_pose_pub = rospy.Publisher("/pf/particle_poses", PoseArray, queue_size=1, latch=True)
        self.obstacles_viz_pub = rospy.Publisher("/pf/obstacles_viz", Marker, queue_size=1, latch=True)

        rospy.Subscriber("/obstacles/coefficients", ModelCoefficientsArray, self.receive_obstacles, queue_size=1)

        # Create the filter object to do our filtering
        self.particle_filter = ParticleFilter(self.world, [1, 1, 0.1, 0.2, 0.2, 0.2], self.n_particles)

        self.publish_particles_poses(rospy.Time.now())
        self.publish_obstacles_viz()

        rospy.spin()

    def receive_obstacles(self, msg):
        measurements = []

        # Read all obstacle x and y from the list
        for i in range(len(msg.values) / step):
            obs = [msg.values[msg.step * i], msg.values[msg.step * i + 1]]
            measurements.append(obs)

        time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1E-9

        if self.last_time < 0:
            self.last_time = time
            return

        self.propogate_state_step(vel, angular_vel, time - self.last_time)
        self.resample_step(obs_x, obs_y)

        self.publish_particles_poses(msg.header.stamp)

    def publish_particles_poses(self, timestamp):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = timestamp

        for p in self.particle_filter.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.position.z = 0

            q = quaternion_from_euler(0.0, 0.0, p.theta)
            pose.orientation = Quaternion(*q)

            msg.poses.append(pose)

        self.particles_pose_pub.publish(msg)

    def publish_obstacles_viz(self):
        i = 0
        for obs in self.known_obstacles:
            # Create visualization
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"

            marker.ns = "pf"  # Set namespace
            marker.id = i  # Give unique id

            marker.type = Marker.SPHERE

            marker.pose = Pose(position=Point(obs[0], obs[1], 0))
            marker.pose.orientation.w = 1.0

            diameter = 0.5
            marker.scale = Vector3(diameter, diameter, diameter)

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1

            # Publish the marker
            self.obstacles_viz_pub.publish(marker)

            i += 1
