#!/usr/bin/env python3
import numpy as np

import rospy
from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, Vector3
from visualization_msgs.msg import Marker

from glenn_msgs.msg import ModelCoefficientsArray


class ParticleFilterLocalization:
    def __init__(self):
        rospy.init_node("partice_filter_localization")
        rospy.loginfo("Particle filter node initialized")

        self.known_obstacles = np.array([[0, 1.5]])
        self.num_particles = 200
        self.vel_noise_sigma = 0.04
        self.angular_vel_noise_sigma = 0.04
        self.half_fov = 0.741
        self.max_dist = 3

        # Particles state = [x, y, theta, vel, theta_dot]
        self.particles = np.empty((0, 5))

        # To do our own odom from wheels + gyro
        self.linear_sum = 0
        self.angular_sum = 0
        self.last_time_imu = -1
        self.last_time_odom = -1
        self.last_linear_sum = 0
        self.last_angular_sum = 0
        self.last_time = -1


        self.particles_pose_pub = rospy.Publisher("/pf/particle_poses", PoseArray, queue_size=1, latch=True)
        self.obstacles_viz_pub = rospy.Publisher("/pf/obstacles_viz", Marker, queue_size=1, latch=True)

        rospy.Subscriber("/imu", Imu, self.receive_imu, queue_size=1)
        rospy.Subscriber("/glenn_base/odom", Odometry, self.receive_odom, queue_size=1)
        rospy.Subscriber("/obstacles/coefficients", ModelCoefficientsArray, self.receive_obstacles, queue_size=1)

        self.initialize_particles(self.num_particles)
        self.publish_particles_poses(rospy.Time.now())
        self.publish_obstacles_viz()

        rospy.spin()

    def receive_imu(self, msg):
        time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1E-9

        if self.last_time_imu < 0:
            self.last_time_imu = time
            return

        self.angular_sum += msg.angular_velocity.z * (time - self.last_time_imu)
        self.last_time_imu = time

    def receive_odom(self, msg):
        time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1E-9

        if self.last_time_odom < 0:
            self.last_time_odom = time
            return

        self.linear_sum += msg.twist.twist.linear.x * (time - self.last_time_odom)
        self.last_time_odom = time

    def initialize_particles(self, num_particles=100):
        self.particles = np.empty((num_particles, 5))

        for i in range(num_particles):
            self.particles[i][0] = 2 * np.random.uniform()
            self.particles[i][1] = 2 * np.random.uniform()
            self.particles[i][2] = 6.28 * np.random.uniform()
            self.particles[i][3] = 0
            self.particles[i][4] = 0

            # self.particles[i][0] = 0
            # self.particles[i][1] = 0
            # self.particles[i][2] = 0

    def receive_obstacles(self, msg):
        obs_x = None
        obs_y = None

        if len(msg.values) != 0:
            # Read obstacle x and y
            obs_x = msg.values[0]
            obs_y = msg.values[1]

        time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1E-9

        if self.last_time < 0:
            self.last_time = time
            return

        # Calculate robot motion model
        vel = (self.linear_sum - self.last_linear_sum) / (time - self.last_time)
        angular_vel = (self.angular_sum - self.last_angular_sum) / (time - self.last_time)

        self.propogate_state_step(vel, angular_vel, time - self.last_time)
        self.resample_step(obs_x, obs_y)

        self.last_angular_sum = self.angular_sum
        self.last_linear_sum = self.linear_sum
        self.last_time = time

        self.publish_particles_poses(msg.header.stamp)

    def publish_particles_poses(self, timestamp):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = timestamp

        for particle in self.particles:
            pose = Pose()
            pose.position.x = particle[0]
            pose.position.y = particle[1]
            pose.position.z = 0

            q = quaternion_from_euler(0.0, 0.0, particle[2])
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

    def propogate_state_step(self, vel, angular_vel, dt):
        # print("%.2f, %.2f, %.2f" % (vel, angular_vel, dt))

        for p in self.particles:
            dtheta = (angular_vel + np.random.normal() * self.angular_vel_noise_sigma) * dt

            if abs(dtheta) < 0.0001:
                dx = (vel + np.random.normal() * self.vel_noise_sigma) * dt
                dy = 0
            else:
                # theta * r = distance around circle
                radius = (vel + np.random.normal() * self.vel_noise_sigma) * dt / dtheta
                dx = radius * np.sin(dtheta)
                dy = radius * (1 - np.cos(dtheta))

            # rotate translation in base_frame to global_frame
            p[0] += dx * np.cos(p[2]) - dy * np.sin(p[2])
            p[1] += dx * np.sin(p[2]) + dy * np.cos(p[2])
            p[2] += dtheta

    def resample_step(self, obs_x, obs_y):
        """
        Resample particles based on posterior distribution
        Step 1: Find p(z|x) for each particle
        Step 2: Sample particles with replacement according to p(z|x)
        """
        probs = self.particles_prob_from_measurement(obs_x, obs_y)

        resampled_particles = np.empty((self.num_particles, 5))
        for i in range(self.num_particles):
            resampled_particles[i] = self.particles[self.random_weighted_index(probs)]

        self.particles = resampled_particles

    def particles_prob_from_measurement(self, obs_x, obs_y):
        probs = []
        for p in self.particles:
            # Find what would be the measured distance
            # and angle to the obstacle from this particle
            x_dist = np.sqrt((self.known_obstacles[0, 0] - p[0])**2 + (self.known_obstacles[0, 1] - p[1])**2)
            x_theta = np.arctan2(p[1] - self.known_obstacles[0, 1], p[0] - self.known_obstacles[0, 0])
            x_theta = self.wrap_angle(x_theta - p[2])

            prob = 0  # Likelyhood of measurement

            # Check if obstacle should not be in view
            if abs(x_theta) > self.half_fov or x_dist > self.max_dist:
                if obs_x:  # and saw an obstacle
                    prob = 0.05  # False postive
                else:
                    prob = 1.0  # True negative
            else:  # Obstacle should be in view
                if not obs_x:  # and didn't see an obstacle
                    prob = 0.1  # False negative
                else:
                    # Find actual distance and angle
                    z_dist = np.sqrt((obs_x - p[0])**2 + (obs_y - p[1])**2)
                    z_theta = np.arctan2(p[1] - obs_y, p[0] - obs_x)
                    z_theta = self.wrap_angle(z_theta - p[2])

                    # Calculate prob based on measurement difference
                    prob = np.exp(-0.5 * ((x_theta - z_theta)**2 / 0.002 + (x_dist - z_dist)**2 / 0.002))

            probs.append(prob)
        
        probs = probs / np.sum(probs)
        # probs.sort()
        print(probs)

        return probs

    def wrap_angle(self, x):
        return (x + 3.1416) % 6.283 - 3.1416

    def random_weighted_index(self, weights):
        r = np.random.uniform()

        total = 0
        for i in range(len(weights)):
            total += weights[i]
            if r < total:
                return i

        return len(weights) - 1
