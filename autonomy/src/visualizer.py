#!/usr/bin/env python3

import rospy
from autonomy.msg import transitPath, transitControlData
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38
ROBOT_WIDTH = 0.63
ROBOT_LENGTH = 1.3
LENGTH = 750


class Visualizer:
    def __init__(self):
        self.path = []
        self.robot = []
        self.uwb_pose = []
        self.target_vels = []
        self.vels = []
        self.target_ang_vels = []
        self.ang_vels = []
        self.target_wheel_speeds = [[0, 0]]
        self.wheel_speeds = [[0, 0]]
        self.followed_segment = np.array([[0, 0]])
        self.closest_point = []
        self.reference_point = []
        self.global_grid = [[0, 50], [100, 0]]
        self.grid_extent = (0, 1, 0, 1)

        print("Opening visualizer")
        rospy.init_node("visualizer_node", anonymous=False)
        rospy.on_shutdown(self.close_plots)

        self.subscribe()

        self.createPlots()

    def subscribe(self):
        rospy.Subscriber("odometry/filtered_map", Odometry, self.receiveOdometry, queue_size=1)
        rospy.Subscriber("uwb_nodes", PoseWithCovarianceStamped, self.recieveUwb, queue_size=1)
        rospy.Subscriber("transitPath", transitPath, self.receivePath, queue_size=1)
        rospy.Subscriber("transitControlData", transitControlData, self.receiveControlData, queue_size=1)
        rospy.Subscriber("global_occupancy_grid", OccupancyGrid, self.recieveOccupancyGrid, queue_size=1)

    def receiveOdometry(self, msg):
        pose = msg.pose.pose
        o = pose.orientation
        angle = R.from_quat([o.x, o.y, o.z, o.w]).as_euler('zyx')[0]
        self.robot = np.array([pose.position.x, pose.position.y, angle]).reshape(3, 1)

    def recieveUwb(self, msg):
        pose = msg.pose.pose
        self.uwb_pose = [pose.position.x, pose.position.y]

    def receivePath(self, msg):
        self.path = np.array(list(msg.path)).reshape((-1, 2))

    def receiveControlData(self, msg):
        self.target_vels.append(msg.t_vel)
        self.target_ang_vels.append(msg.t_angular_vel)
        self.vels.append(msg.vel)
        self.ang_vels.append(msg.angular_vel)
        self.target_wheel_speeds.append([msg.t_right_speed, msg.t_left_speed])
        self.wheel_speeds.append([msg.right_speed, msg.left_speed])

        self.target_vels = self.target_vels[-LENGTH:]
        self.target_ang_vels = self.target_ang_vels[-LENGTH:]
        self.vels = self.vels[-LENGTH:]
        self.ang_vels = self.ang_vels[-LENGTH:]
        self.target_wheel_speeds = self.target_wheel_speeds[-LENGTH:]
        self.wheel_speeds = self.wheel_speeds[-LENGTH:]
        self.followed_segment = np.array(list(msg.followed_segment)).reshape((-1, 2))
        self.closest_point = list(msg.closest_point)
        self.reference_point = list(msg.reference_point)

    def recieveOccupancyGrid(self, msg):
        width = msg.info.width
        height = msg.info.height
        scale = msg.info.resolution
        self.grid_extent = (0, scale * width, 0, scale*height)
        self.global_grid = np.reshape(msg.data, (height, width))

    @staticmethod
    def close_plots():
        print("Closing  Visualizer")
        plt.close()

    def createPlots(self):
        fig = plt.figure(figsize=(12, 4))

        ax1 = plt.subplot2grid((1, 3), (0, 0))
        ax1.axis('equal')
        ax1.set_xlim([-0.1, ARENA_WIDTH + 0.1])
        ax1.set_ylim([-0.1, ARENA_HEIGHT + 0.1])
        ax1.plot([0, ARENA_WIDTH, ARENA_WIDTH, 0, 0],
                 [0, 0, ARENA_HEIGHT, ARENA_HEIGHT, 0], color='black', linewidth=0.5)  # draw field

        line_path, = ax1.plot([], [], linewidth=0.75, color='blue')
        line_robot, = ax1.plot([], [], linewidth=0.75, color='blue')
        line_segment, = ax1.plot([], [], linewidth=0.75, color='orange')
        reference_point, = ax1.plot([], [], marker='o', markersize=3)
        closest_point, = ax1.plot([], [], marker='o', markersize=3)
        img = ax1.imshow(self.global_grid, cmap='Reds', extent=self.grid_extent, vmin=0, vmax=100)
        uwb, = ax1.plot([], [], marker='+',  color='blue', markersize=6)

        ax1.set_title("Arena")

        ax2 = plt.subplot2grid((2, 3), (0, 1), colspan=2)
        ax2.set_xlim([0, LENGTH])
        ax2.set_ylim([-0.2, 0.2])
        line_t_vels, = ax2.plot(self.target_vels, label='target vels')
        line_vels, = ax2.plot(self.vels, label='vels')

        ax2_2 = ax2.twinx()
        ax2_2.set_ylim([-0.9, 0.9])
        line_t_ang_vels, = ax2_2.plot(self.target_ang_vels, label='target ang vels', color='green')
        line_ang_vels, = ax2_2.plot(self.ang_vels, label='ang vels', color='red')
        ax2.set_title('Vels')
        plt.legend(handles=[line_t_vels, line_vels, line_t_ang_vels, line_ang_vels], prop={'size': 7})
        # ax2.legend(prop={'size': 7})
        # ax2_2.legend(prop={'size': 7})

        ax3 = plt.subplot2grid((2, 3), (1, 1), colspan=2)
        ax3.set_xlim([0, LENGTH])
        ax3.set_ylim([-15, 5])
        line_t_wheel_r, line_t_wheel_l = ax3.plot(self.target_wheel_speeds, label='target wheels')
        line_wheel_r, line_wheel_l, = ax3.plot(self.wheel_speeds, label='wheels')
        ax3.set_title('Wheels')
        ax3.legend(prop={'size': 7})

        def animate(i):
            if len(self.path) > 0:
                line_path.set_xdata(self.path[:, 0])
                line_path.set_ydata(self.path[:, 1])

            if len(self.robot) > 0:
                points = SkidSteerSimulator.draw(self.robot, ROBOT_WIDTH, ROBOT_LENGTH)
                points = np.vstack((points, points[0]))  # Make line connect from last point to first
                line_robot.set_xdata(points[:, 0])
                line_robot.set_ydata(points[:, 1])

            line_segment.set_xdata(self.followed_segment[:, 0])
            line_segment.set_ydata(self.followed_segment[:, 1])
            if len(self.closest_point) > 0:
                closest_point.set_xdata(self.closest_point[0])
                closest_point.set_ydata(self.closest_point[1])
            if len(self.reference_point) > 0:
                reference_point.set_xdata(self.reference_point[0])
                reference_point.set_ydata(self.reference_point[1])
            if len(self.uwb_pose) > 0:
                uwb.set_xdata(self.uwb_pose[0])
                uwb.set_ydata(self.uwb_pose[1])

            img.set_data(self.global_grid)
            img.set_extent(self.grid_extent)
            # img.autoscale()

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

            return line_path, line_robot, line_t_vels, line_vels, line_t_ang_vels, line_ang_vels, \
                    line_t_wheel_r, line_t_wheel_l, line_wheel_r, line_wheel_l, img, line_segment, \
                    closest_point, reference_point, uwb

        ani = animation.FuncAnimation(fig, animate, blit=True, interval=50)
        plt.show()


if __name__ == "__main__":
    try:
        visualizer = Visualizer()
    except rospy.ROSInterruptException:
        pass
