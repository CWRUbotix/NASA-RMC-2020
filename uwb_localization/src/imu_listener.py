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
from hci.msg import sensorValue



class IMU:
    def __init__(self):
        self.topic = 'imu'
        print('Booting up node...')
        rospy.init_node('imu_listener', anonymous=True)
        self.orientation = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.acceleration = np.zeros(3)

    def sensor_callback(self, msg):
        if msg.sensorID == 30:
            self.orientation[0] = msg.value
        elif msg.sensorID == 31:
            self.orientation[1] = msg.value
        elif msg.sensorID == 32:
            self.orientation[2] = msg.value
        elif msg.sensorID == 17:
            self.angular_velocity[0] = msg.value
        elif msg.sensorID == 18:
            self.angular_velocity[1] = msg.value
        elif msg.sensorID == 19:
            self.angular_velocity[2] = msg.value
        elif msg.sensorID == 20:
            self.acceleration[0] = msg.value
        elif msg.sensorID == 21:
            self.acceleration[1] = msg.value
        elif msg.sensorID == 22:
            self.acceleration[2] = msg.value

        self.compose_imu_msg()
        self.compose_wheel_msg()

    def compose_imu_msg(self):
        header = Header()
        orientation_quat = R.from_euler('xyz', self.orientation).as_quat()
        orientation_cov = np.ones(9) * (0.00017 ** 2)
        quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        angular_vel_msg = Vector3(self.angular_velocity[0], self.angular_velocity[1], self.angular_velocity[2])
        angular_vel_cov = np.ones(9) * (0.00017 ** 2)
        accel_msg = Vector3(self.acceleration[0], self.acceleration[1], self.acceleration[2])
        accel_cov = np.ones(9) * (0.00017 ** 2)
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom'
        imu_msg = Imu()
        imu_msg.header = header
        imu_msg.orientation = quat_msg
        imu_msg.orientation_covariance = orientation_cov
        imu_msg.angular_velocity = angular_vel_msg
        imu_msg.angular_velocity_covariance = angular_vel_cov
        imu_msg.linear_acceleration = accel_msg
        imu_msg.linear_acceleration_covariance = accel_cov
        try:
            pub = rospy.Publisher('imu', Imu, queue_size=10)
            rospy.loginfo(imu_msg)
            pub.publish(imu_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass

    def compose_wheel_msg(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom'
        point_msg = Point(0, 0, 0)  # use most recent pos with no z-coord
        orientation_quat = R.from_euler('xyz', [0, 0, 0]).as_quat()  # pitch is rotation about z-axis in euler angles
        pose_cov = np.ones(36) * 0
        quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = Pose(point_msg, quat_msg)
        pose_with_cov.covariance = pose_cov

        linear_twist = Vector3(0, 0, 0)
        angular_twist = Vector3(0, 0, 0)

        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist = Twist(linear_twist, angular_twist)
        twist_with_cov.covariance = pose_cov

        stamped_msg = Odometry()
        stamped_msg.header = header
        stamped_msg.child_frame_id = 'base_link'
        stamped_msg.pose = pose_with_cov
        stamped_msg.twist = twist_with_cov
        try:
            pub = rospy.Publisher('wheel', Odometry, queue_size=10)
            rospy.loginfo(stamped_msg)
            pub.publish(stamped_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass




if __name__ == '__main__':
    imu_node = IMU()
    sub = rospy.Subscriber('sensorValue', sensorValue, imu_node.sensor_callback)
    rospy.spin()
