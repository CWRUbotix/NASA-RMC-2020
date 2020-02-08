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
        self.viz_dir = 'imu_plots'
        self.save_plots = True
        print('Booting up node...')
        rospy.init_node('imu_listener', anonymous=True)
        self.orientation = np.zeros(3)
        self.orientation_marker = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.angular_velocity_marker = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.acceleration_marker = np.zeros(3)
        self.port_encoder = 0
        self.starboard_encoder = 0

        self.orientation_plot = np.zeros(3)
        self.angular_velocity_plot = np.zeros(3)
        self.acceleration_plot = np.zeros(3)

        self.angular_velocity_offset = [-4.5, 5.87, 8.37]
        self.acceleration_offset = [0, 0]


        os.makedirs(self.viz_dir, exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

    def sensor_callback(self, msg):
        if msg.sensorID == 0:
            self.port_encoder = msg.value
        if msg.sensorID == 1:
            self.starboard_encoder = msg.value
        if msg.sensorID == 30:
            self.orientation[0] = msg.value
            self.orientation_marker[0] = 1
        elif msg.sensorID == 31:
            self.orientation[1] = msg.value
            self.orientation_marker[1] = 1
        elif msg.sensorID == 32:
            self.orientation[2] = msg.value
            self.orientation_marker[2] = 1
        elif msg.sensorID == 17:
            self.angular_velocity[0] = msg.value
            self.angular_velocity_marker[0] = 1
        elif msg.sensorID == 18:
            self.angular_velocity[1] = msg.value
            self.angular_velocity_marker[1] = 1
        elif msg.sensorID == 19:
            self.angular_velocity[2] = msg.value
            self.angular_velocity_marker[2] = 1
        elif msg.sensorID == 20:
            self.acceleration[0] = msg.value
            self.acceleration_marker[0] = 1
        elif msg.sensorID == 21:
            self.acceleration[1] = msg.value
            self.acceleration_marker[1] = 1
        elif msg.sensorID == 22:
            self.acceleration[2] = msg.value
            self.acceleration_marker[2] = 1

        if self.orientation_marker.all() == 1 and self.angular_velocity_marker.all() == 1 and self.acceleration_marker.all() == 1:
            self.compose_imu_msg()
            self.compose_wheel_msg()

            self.orientation_plot = np.vstack((self.orientation_plot, self.orientation))
            self.angular_velocity_plot = np.vstack((self.angular_velocity_plot, self.angular_velocity))
            self.acceleration_plot = np.vstack((self.acceleration_plot, self.acceleration))

            self.orientation_marker = np.zeros(3)
            self.angular_velocity_marker = np.zeros(3)
            self.acceleration_marker = np.zeros(3)
            print('X: %.4f' % np.var(self.angular_velocity_plot[..., 0]), 'Y: %.4f' % np.var(self.angular_velocity_plot[..., 1]), 'Z: %.4f' % np.var(self.angular_velocity_plot[..., 2]))

            if self.orientation_plot.shape[0] % 5 == 0 and self.save_plots:
                fig = plt.figure(figsize=(15, 5))
                ax = plt.subplot(131)
                ax.plot(self.orientation_plot[..., 0], label='x')
                ax.plot(self.orientation_plot[..., 1], label='y')
                ax.plot(self.orientation_plot[..., 2], label='z')
                ax.set_title('Orientation')
                ax.legend(loc='best')

                ax = plt.subplot(132)
                ax.plot(self.angular_velocity_plot[..., 0] + self.angular_velocity_offset[0], label='x')
                ax.plot(self.angular_velocity_plot[..., 1] + self.angular_velocity_offset[1], label='y')
                ax.plot(self.angular_velocity_plot[..., 2] + self.angular_velocity_offset[2], label='z')
                ax.set_title('Angular Velocity')
                ax.legend(loc='best')

                ax = plt.subplot(133)
                ax.plot(self.acceleration_plot[..., 0], label='x')
                ax.plot(self.acceleration_plot[..., 1], label='y')
                ax.plot(self.acceleration_plot[..., 2], label='z')
                ax.set_title('Acceleration')
                ax.legend(loc='best')

                fig.savefig(self.viz_dir + '/imu_%d.png' % (len(os.listdir(self.viz_dir))))
                plt.close()

    def compose_imu_msg(self):
        header = Header()
        orientation_quat = R.from_euler('xyz', self.orientation).as_quat()
        orientation_cov = np.ravel(np.eye(3) * 1e-9)
        quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        angular_vel_msg = Vector3(self.angular_velocity[0] + self.angular_velocity_offset[0],
                                  self.angular_velocity[1] + self.angular_velocity_offset[1],
                                  self.angular_velocity[2] + self.angular_velocity_offset[2])
        angular_vel_cov = np.ravel(np.eye(3) * 1e-9)
        accel_msg = Vector3(-(self.acceleration[0] + self.acceleration_offset[0]),
                            -(self.acceleration[1] + self.acceleration_offset[1]),
                            -self.acceleration[2])
        accel_cov = np.ravel(np.eye(3) * 1e-9)
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
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
            #rospy.loginfo(imu_msg)
            pub.publish(imu_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass

    def compose_wheel_msg(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        #print('Port:', self.port_encoder, 'Starboard:', self.starboard_encoder)
        point_msg = Point(0, 0, 0)
        orientation_quat = R.from_euler('xyz', [0, 0, 0]).as_quat()
        pose_cov = np.ravel(np.eye(6) * 0.0)
        quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = Pose(point_msg, quat_msg)
        pose_with_cov.covariance = pose_cov

        x_dot = (((self.port_encoder * math.pi / 30 * 0.2286) + (self.starboard_encoder * math.pi / 30 * 0.2286)) / 2)# * math.cos(self.orientation[-1])
        y_dot = 0 #(((self.port_encoder * math.pi / 30 * 0.2286) + (self.starboard_encoder * math.pi / 30 * 0.2286)) / 2) * math.sin(self.orientation[-1])
        theta_dot = ((self.starboard_encoder * math.pi / 30 * 0.2286) - (self.port_encoder * math.pi / 30 * 0.2286)) / 0.63

        linear_twist = Vector3(x_dot, y_dot, 0)
        angular_twist = Vector3(0, 0, theta_dot)

        twist_cov = np.diag([0.05, 0.05, 0, 0, 0, 0.05]).flatten()
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
            #rospy.loginfo(stamped_msg)
            pub.publish(stamped_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass




if __name__ == '__main__':
    imu_node = IMU()
    sub = rospy.Subscriber('sensorValue', sensorValue, imu_node.sensor_callback)
    rospy.spin()
