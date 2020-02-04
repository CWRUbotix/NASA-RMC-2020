#!/usr/bin/env python3

import rospy
from autonomy.msg import transitPath, transitControlData
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38
ROBOT_WIDTH = 0.63
ROBOT_LENGTH = 1.3
effective_robot_width = 0.7
reference_point_x = 0.3


class Visualizer:
    def __init__(self):
        self.path = []
        self.robot = []
        self.target_vels = []
        self.vels = []
        self.target_ang_vels = []
        self.ang_vels = []
        self.target_wheel_speeds = [[0, 0]]
        self.wheel_speeds = [[0, 0]]

        print("Opening visualizer")
        rospy.init_node("visualizer_node", anonymous=False)
        self.subscribe()

        self.createPlots()

    def subscribe(self):
        rospy.Subscriber("odometry/filtered_map", Odometry, self.receiveOdometry)
        rospy.Subscriber("transitPath", transitPath, self.receivePath)
        rospy.Subscriber("transitControlData", transitControlData, self.recieveControlData)

    def receiveOdometry(self, msg):
        pose = msg.pose.pose
        o = pose.orientation
        angle = R.from_quat([o.x, o.y, o.z, o.w]).as_euler('zyx')[0]
        self.robot = np.array([pose.position.x, pose.position.y, angle]).reshape(3, 1)

    def receivePath(self, msg):
        self.path = np.array(list(msg.path)).reshape((-1, 2))

    def recieveControlData(self, msg):
        length = 200
        self.target_vels.append(msg.t_vel)
        self.target_ang_vels.append(msg.t_angular_vel)
        self.vels.append(msg.vel)
        self.ang_vels.append(msg.angular_vel)
        self.target_wheel_speeds.append([msg.t_right_speed, msg.t_left_speed])
        self.wheel_speeds.append([msg.right_speed, msg.left_speed])

        self.target_vels = self.target_vels[-length:]
        self.target_ang_vels = self.target_ang_vels[-length:]
        self.vels = self.vels[-length:]
        self.ang_vels = self.ang_vels[-length:]
        self.target_wheel_speeds = self.target_wheel_speeds[-length:]
        self.wheel_speeds = self.wheel_speeds[-length:]

    def createPlots(self):
        fig = plt.figure(figsize=(12, 4))

        ax1 = plt.subplot(121)
        ax1.axis('equal')
        ax1.set_xlim([-2, 6])
        ax1.set_ylim([-1, 8])
        ax1.plot([0, ARENA_WIDTH, ARENA_WIDTH, 0, 0],
                 [0, 0, ARENA_HEIGHT, ARENA_HEIGHT, 0], color='black', linewidth=0.5)  # draw field

        line_path, = ax1.plot([], [], linewidth=0.75, color='blue')
        line_robot, = ax1.plot([], [], linewidth=0.75, color='blue')

        ax1.set_title("Arena")

        ax2 = plt.subplot(222)
        ax2.set_xlim([0, 200])
        ax2.set_ylim([-3, 3])
        line_t_vels, = ax2.plot(self.target_vels, label='target vels')
        line_vels, = ax2.plot(self.vels, label='vels')
        line_t_ang_vels, = ax2.plot(self.target_ang_vels, label='target ang vels')
        line_ang_vels, = ax2.plot(self.ang_vels, label='ang vels')
        ax2.set_title('Vels')
        ax2.legend()

        ax3 = plt.subplot(224)
        ax3.set_xlim([0, 200])
        ax3.set_ylim([-30, 30])
        line_t_wheel_r, line_t_wheel_l = ax3.plot(self.target_wheel_speeds, label='target wheels')
        line_wheel_r, line_wheel_l, = ax3.plot(self.wheel_speeds, label='wheels')
        ax3.set_title('Wheels')
        ax3.legend()

        def animate(i):
            if len(self.path) > 0:
                line_path.set_xdata(self.path[:, 0])
                line_path.set_ydata(self.path[:, 1])

            if len(self.robot) > 0:
                points = SkidSteerSimulator.draw(self.robot, ROBOT_WIDTH, ROBOT_LENGTH)
                points = np.vstack((points, points[0]))  # Make line connect from last point to first
                line_robot.set_xdata(points[:, 0])
                line_robot.set_ydata(points[:, 1])

            length = len(self.target_vels)
            x_values = np.arange(length)

            line_t_vels.set_ydata(self.target_vels[:length])
            line_vels.set_ydata(self.vels[:length])
            line_t_ang_vels.set_ydata(self.target_ang_vels[:length])
            line_ang_vels.set_ydata(self.ang_vels[:length])
            line_t_wheel_r.set_ydata(np.array(self.target_wheel_speeds)[:length, 0])
            line_t_wheel_l.set_ydata(np.array(self.target_wheel_speeds)[:length, 1])
            line_wheel_r.set_ydata(np.array(self.wheel_speeds)[:length, 0])
            line_wheel_l.set_ydata(np.array(self.wheel_speeds)[:length, 1])

            line_t_vels.set_xdata(x_values)
            line_vels.set_xdata(x_values)
            line_t_ang_vels.set_xdata(x_values)
            line_ang_vels.set_xdata(x_values)
            line_t_wheel_r.set_xdata(x_values)
            line_t_wheel_l.set_xdata(x_values)
            line_wheel_r.set_xdata(x_values)
            line_wheel_l.set_xdata(x_values)

            return line_path, line_robot, line_t_vels, line_vels, line_t_ang_vels, line_ang_vels\
                    , line_t_wheel_r, line_t_wheel_l, line_wheel_r, line_wheel_l

        ani = animation.FuncAnimation(fig, animate, blit=True, interval=50)
        plt.show()


if __name__ == "__main__":
    try:
        visualizer = Visualizer()
    except rospy.ROSInterruptException:
        pass
