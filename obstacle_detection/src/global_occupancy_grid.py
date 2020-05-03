#!/usr/bin/env python3

import os
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')  # Fix cv2 import error
import cv2  # TODO Dumb fix please fix
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')  # Fix cv2 import error

import glob

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
from scipy import ndimage

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion


class GlobalOccupancyGrid:

    def __init__(self):

        print('Booting up node...')
        rospy.init_node('global_map', anonymous=True)
        self.save_imgs = False
        self.save_data = False
        self.robot_x = []
        self.robot_y = []
        self.robot_pitch = []
        self.local_grid = None
        self.arena_length = rospy.get_param('arena_y')
        self.arena_width = rospy.get_param('arena_x')
        self.resolution = rospy.get_param('obstacle_detection/grid_resolution')
        self.camera_offset = [rospy.get_param('obstacle_detection/realsense/x'),
                              rospy.get_param('obstacle_detection/realsense/y'), rospy.get_param('obstacle_detection/realsense/yaw')]
        self.global_grid_shape = (int(self.arena_length / self.resolution), int(self.arena_width / self.resolution))
        self.global_grid = np.zeros(self.global_grid_shape)
        self.global_counts = np.zeros_like(self.global_grid)  # keeps track of number of measurements for each cell
        self.global_totals = np.zeros_like(self.global_grid)  # keeps track of sum of measurements for each cell
        self.local_grid_topic = 'local_occupancy_grid'
        self.viz_dir = 'global_map/'
        self.data_dir = 'occupancy_grid_data/'

        os.makedirs(self.viz_dir, exist_ok=True)
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.data_dir + 'localization', exist_ok=True)

        self.clear_dir(self.viz_dir)
        self.clear_dir(self.data_dir)
        self.clear_dir(self.data_dir + 'localization')

    @staticmethod
    def clear_dir(dir_name):
        files = glob.glob('%s/*' % (dir_name))
        for f in files:
            try:
                os.remove(f)
            except OSError as e:
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
        #print('X: %.4f \tY: %.4f \tpitch: %.4f' % (self.robot_x[-1], self.robot_y[-1], self.robot_pitch[-1]))

    def has_localization_data(self):
        return len(self.robot_x) > 0 and len(self.robot_y) > 0 and len(self.robot_pitch) > 0

    def paste_slices(self, tup):
        pos, w, max_w = tup
        wall_min = max(pos, 0)
        wall_max = min(pos + w, max_w)
        block_min = -min(pos, 0)
        block_max = max_w - max(pos + w, max_w)
        block_max = block_max if block_max != 0 else None
        return slice(wall_min, wall_max), slice(block_min, block_max)

    def paste(self, wall, block, loc):
        loc_zip = zip(loc, block.shape, wall.shape)
        wall_slices, block_slices = zip(*map(self.paste_slices, loc_zip))

        wall_copy = -1 * np.ones_like(wall)
        try:
            wall_copy[wall_slices] = block[block_slices]
        except ValueError:
            pass

        return wall_copy

    def local_grid_callback(self, msg):
        grid_size = msg.info.width
        self.local_grid = np.reshape(msg.data, (grid_size, grid_size))

        if self.has_localization_data():
            if self.save_data:
                print('Saving...')
                np.save('%s/%d.npy' % (self.data_dir, len(os.listdir(self.data_dir))), self.local_grid)
                np.save('%s/%d.npy' % (self.data_dir + 'localization', len(os.listdir(self.data_dir + 'localization'))),
                        np.array([self.robot_x[-1], self.robot_y[-1], self.robot_pitch[-1]]))
            indices = np.indices(self.local_grid.shape)
            mask = abs(indices[1] - self.local_grid.shape[1] / 2) < (self.local_grid.shape[0] - indices[0])
            mask = np.reshape(mask, (grid_size, grid_size))
            self.local_grid[~mask] = -1
            #  Rotate image using degrees for some dumb reason
            rotated_grid = ndimage.rotate(self.local_grid, (self.robot_pitch[-1] * 180 / np.pi + self.camera_offset[2]),
                                          mode='constant', cval=-1, reshape=True)  # rotate grid by camera + robot angle

            # location of center of top down grid image in camera coordinate frame, in meters
            local_origin = [0, grid_size / 2 * msg.info.resolution]

            angle = self.camera_offset[2] * np.pi / 180  # Rotate to transform to robot coordinate frame
            local_origin = np.array([[np.cos(angle), -np.sin(angle)],
                                     [np.sin(angle), np.cos(angle)]]).dot(local_origin)
            local_origin = local_origin + self.camera_offset[:2]  # translate to convert to robot coordinate frame

            angle = self.robot_pitch[-1]  # Rotate to transform to global coordinate frame
            local_origin = np.array([[np.cos(angle), -np.sin(angle)],
                                     [np.sin(angle), np.cos(angle)]]).dot(local_origin)
            local_origin = local_origin + [self.robot_x[-1], self.robot_y[-1]]  # translate to global coordinate frame

            # convert to grid indices:
            local_origin = local_origin / self.resolution

            # flip due to coordinate system of matrix being top left
            # Switch local origin from center of image to top left based on new width of rotated image
            local_origin = [int(np.round(self.global_grid_shape[0] - local_origin[1] - rotated_grid.shape[0] / 2)),
                            int(np.round(local_origin[0] - rotated_grid.shape[1] / 2))]

            print(local_origin)

            counts = np.ones_like(rotated_grid)  # Count how many times each cell was measured
            counts[rotated_grid == -1] = -1
            counts = self.paste(self.global_counts, counts, local_origin)

            roi = (counts != -1)

            counts[~roi] = 0

            global_additions = self.paste(self.global_totals, rotated_grid, local_origin)
            window_size = 100
            self.global_totals[roi] = (self.global_totals[roi] * np.minimum(self.global_counts[roi], window_size) + global_additions[roi]) \
                                      / (np.minimum(self.global_counts[roi], window_size) + 1)

            self.global_counts += counts

            # self.global_grid = np.divide(self.global_totals, self.global_counts, out=np.zeros_like(self.global_totals), where=self.global_counts!=0)
            # self.global_grid = np.nan_to_num(self.global_grid, copy=False)
            # self.global_grid /= np.max(self.global_grid)  # ensure values are 0-1

            self.global_grid = ndimage.maximum_filter(self.global_totals, size=6, mode='nearest')
            self.global_grid[self.global_grid >= 50] = 100
            self.global_grid = ndimage.gaussian_filter(self.global_grid, sigma=1.5)
            self.global_grid[self.global_grid >= 50] = 100

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'

            map_meta_data = MapMetaData()
            map_meta_data.map_load_time = rospy.Time.now()
            map_meta_data.resolution = self.resolution  # each cell is 15cm
            map_meta_data.width = self.global_grid_shape[1]
            map_meta_data.height = self.global_grid_shape[0]
            map_meta_data.origin = Pose(Point(0, 0, 0),
                                        Quaternion(0, 0, 0, 1))

            grid_msg = OccupancyGrid()
            grid_msg.header = header
            grid_msg.info = map_meta_data
            grid_msg.data = list(np.int8(self.global_grid.flatten()))

            try:
                pub = rospy.Publisher(rospy.get_param('obstacle_detection_name'), OccupancyGrid, queue_size=1)
                pub.publish(grid_msg)
            except rospy.ROSInterruptException as e:
                print(e.getMessage())
                pass

            if self.save_imgs:
                fig = plt.figure(figsize=(15, 5))

                ax = plt.subplot(131)
                ax.imshow(self.local_grid, cmap='Reds', vmin=0, vmax=100)
                ax.set_title('Local Occupancy Grid')

                ax = plt.subplot(132)
                ax.imshow(rotated_grid, cmap='Reds', vmin=0, vmax=100)
                ax.set_title('Rotated Occupancy Grid')

                ax = plt.subplot(133)
                ax.imshow(self.global_grid, cmap='Reds', vmin=0, vmax=100)
                ax.set_title('Global Occupancy Grid')

                fig.savefig('%s/%d' % (self.viz_dir, len(os.listdir(self.viz_dir))))
                plt.close()
        else:
            print("No localization data, can not form global grid")


if __name__ == '__main__':
    try:
        global_grid = GlobalOccupancyGrid()
        rospy.Subscriber(global_grid.local_grid_topic, OccupancyGrid, global_grid.local_grid_callback, queue_size=1)
        rospy.Subscriber("/odometry/filtered_map", Odometry, global_grid.localization_listener, queue_size=1)
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
