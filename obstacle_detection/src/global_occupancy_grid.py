#!/usr/bin/env python3
import os
import sys
import glob
import cv2
import math
import rospy
import math
import matplotlib

matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion


class GlobalOccupancyGrid:

    def __init__(self):
        self.save_imgs = True
        self.save_data = True
        self.robot_x = []
        self.robot_y = []
        self.robot_pitch = []
        self.local_grid = None
        self.arena_length = 5.4
        self.arena_width = 3.6
        self.resolution = 0.15
        self.global_grid_shape = ((self.arena_length / self.resolution), (self.arena_width / self.resolution))
        self.global_grid = np.zeros(self.global_grid_shape)
        self.localization_topic = 'odometry/filtered_map'
        self.local_grid_topic = 'local_occupancy_grid'
        self.viz_dir = 'global_map/'
        self.data_dir = 'occupancy_grid_data/'

        print('Booting up node...')
        rospy.init_node('globalMap', anonymous=True)

        os.makedirs(self.viz_dir, exist_ok=True)
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.data_dir + 'localization', exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)
        try:
            files = glob.glob('%s/*' % self.data_dir + 'localization')
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

    def localization_listener(self, msg):
        pose = msg.pose.pose
        covariance = msg.pose.covariance
        covariance = np.reshape(covariance, (6, 6))
        self.robot_x.append(pose.position.x)
        self.robot_y.append(pose.position.y)
        quat = pose.orientation
        euler = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
        self.robot_pitch.append(euler[2])  # rotation about vertical z-axis
        print('X: %.4f \tY: %.4f \tpitch: %.4f' % (self.robot_x[-1], self.robot_y[-1], self.robot_pitch[-1]))

    def has_localization_data(self):
        return len(self.robot_x) > 0 and len(self.robot_y) > 0 and len(self.robot_pitch) > 0

    def local_grid_callback(self, msg):
        grid_size = msg.info.width
        self.local_grid = np.reshape(msg.data, (grid_size, grid_size))
        if self.save_data:
            np.save('%s/%d.npy' % (self.data_dir, len(os.listdir(self.data_dir))), self.local_grid)
            np.save('%s/%d.npy' % (self.data_dir + 'localization', len(os.listdir(self.data_dir + 'localization'))),
                    np.array(self.robot_x[-1], self.robot_y[-1], self.robot_pitch[-1]))


if __name__ == '__main__':
    try:
        global_grid = GlobalOccupancyGrid()
        rospy.Subscriber(global_grid.local_grid_topic, OccupancyGrid, global_grid.local_grid_callback, queue_size=1)
        rospy.Subscriber(global_grid.localization_topic, Odometry, global_grid.localization_listener, queue_size=1)
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
