#!/usr/bin/env python3
import rospy
import matplotlib
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import matplotlib.pyplot as plt
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Imu


class RealSenseIMU:
    def __init__(self):
        self.gyro_weight = 20
        self.visualize = False
        self.step = 0
        self.window = 20

        self.last_time = 0
        self.last_estimate = np.array([0, 0, 1])
        self.last_gyro = np.array([0, 0, 0])

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.x_line = self.ax.plot([0, 1], [0, 0], [0, 0], label='x')[0]
        self.y_line = self.ax.plot([0, 0], [0, 1], [0, 0], label='y')[0]
        self.z_line = self.ax.plot([0, 0], [0, 0], [0, 1], label='z')[0]

        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_zlim(-1.5, 1.5)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.legend(loc='best')
        #self.ax.hold(True)

        # plt.show(False)
        # plt.draw()

        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)

        print('Booting up node...')
        rospy.init_node('realsense_motion_tracking', anonymous=True)
        self.imu_pub = rospy.Publisher('imu_realsense/data', Imu, queue_size=1)
        rospy.Subscriber("imu_realsense/data_raw", Imu, self.receive_imu_msg)

        rospy.spin()

    def receive_imu_msg(self, msg):
        # integration of IMU data referenced from https://github.com/IntelRealSense/librealsense/issues/4391
        # and http://www.starlino.com/imu_guide.html
        time = msg.header.stamp.to_sec()

        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # different reference frame prevent errors with noise?
        # accel = np.array([-msg.linear_acceleration.z, msg.linear_acceleration.x, -msg.linear_acceleration.y])
        # gyro = np.array([msg.angular_velocity.z, -msg.angular_velocity.x, msg.angular_velocity.y])

        # accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.z, -msg.linear_acceleration.y])
        # gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.z, -msg.angular_velocity.y])

        accel_mag = np.sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2])
        accel = accel / accel_mag

        if self.last_time == 0:
            self.last_time = rospy.Time.now().to_sec()
            self.last_estimate = accel

        # x angle (roll) is rotation in the yz plane
        accel_angle_x = np.arctan2(self.last_estimate[1], self.last_estimate[2])
        accel_angle_y = np.arctan2(self.last_estimate[0], self.last_estimate[2])

        new_angle_x = accel_angle_x + 0.5 * (gyro[0] + self.last_gyro[0]) * (time - self.last_time)
        new_angle_y = accel_angle_y + 0.5 * (gyro[1] + self.last_gyro[1]) * (time - self.last_time)

        gyro_dirs = np.array([0.0, 0.0, 0.0])
        gyro_dirs[0] = np.sign(new_angle_y) / np.sqrt(1 + 1 / (np.tan(new_angle_y) ** 2 * np.cos(new_angle_x) ** 2))
        gyro_dirs[1] = np.sign(new_angle_x) / np.sqrt(1 + 1 / (np.tan(new_angle_x) ** 2 * np.cos(new_angle_y) ** 2))
        gyro_dirs[2] = np.sign(self.last_estimate[2]) * np.sqrt(1 - gyro_dirs[0] ** 2 - gyro_dirs[1] ** 2)

        estimate = (accel + gyro_dirs * self.gyro_weight) / (1 + self.gyro_weight)
        estimate_mag = np.sqrt(estimate[0] * estimate[0] + estimate[1] * estimate[1] + estimate[2] * estimate[2])
        estimate = estimate / estimate_mag

        angles = [np.arctan2(estimate[1], estimate[2]), np.arctan2(estimate[0], estimate[2]), 0]
        print()
        print("{:.3f} {:.3f} {:.3f}".format(accel[0], accel[1], accel[2]))
        print("{:.3f} {:.3f} {:.3f}".format(gyro[0], gyro[1], gyro[2]))
        print("{:.3f} {:.3f} {:.3f}".format(accel_angle_x, accel_angle_y, 0))
        print("{:.3f} {:.3f} {:.3f}".format(new_angle_x, new_angle_y, 0))
        print("{:.3f} {:.3f} {:.3f}".format(gyro_dirs[0], gyro_dirs[1], gyro_dirs[2]))
        print("{:.3f} {:.3f} {:.3f}".format(estimate[0], estimate[1], estimate[2]))

        self.last_estimate = estimate
        self.last_gyro = gyro
        self.last_time = time
        self.step += 1

        try:
            imu_msg = msg
            imu_msg.header.frame_id = "realsense_link"
            quat = R.from_euler('xyz', angles).as_quat()
            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.orientation.w = quat[3]
            self.imu_pub.publish(imu_msg)
        except rospy.ROSInterruptException as e:
            rospy.logerr(e.getMessage())

        if self.step % self.window == 0:
            print("Angles: R {:.3f} P {:.3f} Y {:.3f}".format(angles[0], angles[1], angles[2]))
            if self.visualize :
                r = R.from_euler('zyx', angles)
                x_new = r.apply(np.array([1, 0, 0]))
                y_new = r.apply(np.array([0, 1, 0]))
                z_new = r.apply(np.array([0, 0, 1]))

                # restore background
                self.fig.canvas.restore_region(self.background)

                # redraw just the points
                self.x_line.set_xdata([0, x_new[0]])
                self.x_line.set_ydata([0, x_new[1]])
                self.x_line.set_3d_properties([0, x_new[2]])
                #self.ax.draw_artist(self.x_line)
                self.y_line.set_xdata([0, y_new[0]])
                self.y_line.set_ydata([0, y_new[1]])
                self.y_line.set_3d_properties([0, y_new[2]])
                #self.ax.draw_artist(self.y_line)
                self.z_line.set_xdata([0, z_new[0]])
                self.z_line.set_ydata([0, z_new[1]])
                self.z_line.set_3d_properties([0, z_new[2]])
                #self.ax.draw_artist(self.z_line)

                # fill in the axes rectangle
                self.fig.canvas.blit(self.ax.bbox)
                plt.draw()
                plt.show(block=False)
