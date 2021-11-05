#!/usr/bin/env python3
import os
import glob
import math
import rospy
import rospkg
import math
import matplotlib
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, TwistWithCovariance, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from hwctrl.msg import Encoders


class WHEELODOMETRY:
    def __init__(self):
        self.viz_dir = 'imu_plots'
        self.save_plots = False
        rospy.loginfo('Imu listener node initializing')
        rospy.init_node('imu_listener_node', anonymous=True)
        self.orientation = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.acceleration = np.zeros(3)

        self.wheel_radius = rospy.get_param('wheel_radius')
        self.robot_width = rospy.get_param('effective_robot_width')

        self.orientation_plot = np.zeros(3)
        self.angular_velocity_plot = np.zeros(3)
        self.acceleration_plot = np.zeros(3)

        self.angular_velocity_offset = [rospy.get_param('imu_angular_vel_x_offset'),
                                        rospy.get_param('imu_angular_vel_y_offset'),
                                        rospy.get_param('imu_angular_vel_z_offset')]
        self.acceleration_offset = [0, 0, 0]

        self.wheel_pub = rospy.Publisher('glenn_base/odom', Odometry, queue_size=10)
        
        rospy.Subscriber('glenn_base/encoders', Encoders, self.receive_encoders)

        os.makedirs(self.viz_dir, exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

        rospy.spin()

    
    
    def receive_encoders(self, msg):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom'

        point_msg = Point(0, 0, 0)
        orientation_quat = R.from_euler('xyz', [0, 0, 0]).as_quat()
        pose_cov = np.ravel(np.eye(6) * 0.0)
        quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = Pose(point_msg, quat_msg)
        pose_with_cov.covariance = pose_cov

        x_dot = (((msg.left * math.pi / 30 * self.wheel_radius) + (msg.right * math.pi / 30 * self.wheel_radius)) / 2)# * math.cos(self.orientation[-1])
        y_dot = 0 #(((self.port_encoder * math.pi / 30 * 0.2286) + (self.starboard_encoder * math.pi / 30 * 0.2286)) / 2) * math.sin(self.orientation[-1])
        theta_dot = ((msg.right * math.pi / 30 * self.wheel_radius) - (msg.left * math.pi / 30 * self.wheel_radius)) / self.robot_width

        linear_twist = Vector3(x_dot, y_dot, 0)
        angular_twist = Vector3(0, 0, theta_dot)

        twist_cov = np.diag([0.002, 0.02, 0, 0, 0, 0.001]).flatten()
        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist = Twist(linear_twist, angular_twist)
        twist_with_cov.covariance = twist_cov

        stamped_msg = Odometry()
        stamped_msg.header = header
        stamped_msg.child_frame_id = 'base_footprint'
        stamped_msg.pose = pose_with_cov
        stamped_msg.twist = twist_with_cov

        try:
            self.wheel_pub.publish(stamped_msg)
        except rospy.ROSInterruptException as e:
            rospy.logerr(e.getMessage())


if __name__ == '__main__':
    wheel_odom_node = WHEELODOMETRY()
