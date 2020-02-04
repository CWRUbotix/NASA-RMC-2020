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
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from scipy import signal
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

from realsense_utils import *

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print('Depth scale:', depth_scale)
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()


class ObstacleDetectionNode:

    def __init__(self):
        self.h, self.w = 512, 424
        self.ground_plane_height = 0
        self.resolution = 0.15
        self.grid_size = 30
        self.tolerance = 0.05
        self.kernel_size = 5
        self.save_imgs = True
        self.save_data = True
        self.robot_x = []
        self.robot_y = []
        self.robot_pitch = []
        self.localization_topic = 'odometry/filtered_map'
        self.viz_dir = 'obstacle_viz/'
        self.data_dir = 'saved_frames/'
        # camera information based on the Kinect v2 hardware
        self.CameraParams = {
            "cx": 254.878,
            "cy": 205.395,
            "fx": 365.456,
            "fy": 365.456,
            "k1": 0.0905474,
            "k2": -0.26819,
            "k3": 0.0950862,
            "p1": 0.0,
            "p2": 0.0,
        }
        # Kinect's physical orientation in the real world.
        self.CameraPosition = {
            "x": 0,  # actual position in meters of kinect sensor relative to the viewport's center.
            "y": 0,  # actual position in meters of kinect sensor relative to the viewport's center.
            "z": 0,  # height in meters of actual kinect sensor from the floor.
            "roll": 0,
            # angle in degrees of sensor's roll (used for INU input - trig function for this is commented out by default).
            "azimuth": 0,  # sensor's yaw angle in degrees.
            "elevation": -30,  # sensor's pitch angle in degrees.
        }

        print('Booting up node...')
        rospy.init_node('obstacleDetection', anonymous=True)

        os.makedirs(self.viz_dir, exist_ok=True)
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.data_dir + 'color', exist_ok=True)
        os.makedirs(self.data_dir + 'localization', exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)
        try:
            files = glob.glob('%s/*' % self.data_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)
        try:
            files = glob.glob('%s/*' % (self.data_dir + 'color'))
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)
        try:
            files = glob.glob('%s/*' % (self.data_dir + 'localization'))
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

    def project_point_cloud_onto_plane(self, xyz_arr, resize_factor=10, cropping=500):
        proj = xyz_arr[..., [0, 2]]
        proj_img = np.zeros((4500, 4500))
        indices = np.int32(proj * 1000)
        try:
            indices[..., 1] += 4500 // 2
            indices[..., 0] += 4500 // 2
            indices = np.clip(indices, 0, 4499)
            proj_img[indices[..., 0], indices[..., 1]] = 255
            new_size = 4500 // resize_factor
            proj_img = cv2.resize(proj_img, (new_size, new_size), interpolation=cv2.INTER_AREA)
            proj_img = cv2.dilate(proj_img, np.ones((3, 3)), iterations=2)
            proj_img = cv2.blur(proj_img, (5, 5))
            proj_img = cv2.rotate(proj_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        except ValueError:
            pass
        return np.uint8(proj_img)

    def detect_obstacles_from_above(self, depth_frame, color_frame):
        out = np.empty((h, w, 3), dtype=np.uint8)
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = np.asanyarray(
            colorizer.colorize(depth_frame).get_data())

        if state.color:
            mapped_frame, color_source = color_frame, color_image
        else:
            mapped_frame, color_source = depth_frame, depth_colormap

        points = pc.calculate(depth_frame)
        pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        xyz_arr = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  #

        out.fill(0)

        grid(out, (0, 0.5, 1), size=1, n=10)
        frustum(out, depth_intrinsics)
        axes(out, view([0, 0, 0]), state.rotation, size=0.1, thickness=1)

        if not state.scale or out.shape[:2] == (h, w):
            pointcloud(out, xyz_arr, texcoords, color_source)
        else:
            tmp = np.zeros((h, w, 3), dtype=np.uint8)
            pointcloud(tmp, xyz_arr, texcoords, color_source)
            tmp = cv2.resize(
                tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
            np.putmask(out, tmp > 0, tmp)


        rocks = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 1] >= self.ground_plane_height + self.tolerance])
        holes = self.project_point_cloud_onto_plane(xyz_arr)

        rock_grid = self.gridify(rocks, (self.grid_size, self.grid_size))
        hole_grid = self.gridify(holes, (self.grid_size, self.grid_size))
        obs_grid = np.maximum(rock_grid, hole_grid)
        kernel = np.ones((self.kernel_size, self.kernel_size))

        occupancy_grid = signal.convolve2d(obs_grid, kernel, boundary='symm', mode='same')
        occupancy_grid /= np.max(occupancy_grid)  # ensure values are 0-1

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom'

        map_meta_data = MapMetaData()
        map_meta_data.map_load_time = rospy.Time.now()
        map_meta_data.resolution = self.resolution  # each cell is 15cm
        map_meta_data.width = self.grid_size
        map_meta_data.height = self.grid_size
        map_meta_data.origin = Pose(Point(0, 0, 0),
                                    Quaternion(0, 0, 0, 1))

        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info = map_meta_data
        grid_msg.data = list(np.int8(occupancy_grid.flatten() * 100))

        try:
            pub = rospy.Publisher('local_occupancy_grid', OccupancyGrid, queue_size=1)
            pub.publish(grid_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass

        if self.save_imgs:
            fig = plt.figure(figsize=(20, 5))

            ax = plt.subplot(241)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.imshow(depth_image, cmap='jet')
            ax.set_title('Depth Frame')

            ax = plt.subplot(245)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.imshow(color_image)
            ax.set_title('Color Frame')

            ax = plt.subplot(242)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.imshow(rocks)
            #ax.imshow(out)
            ax.set_title('Projection')

            ax = plt.subplot(243)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.imshow(obs_grid, cmap='Reds')
            ax.set_title('Raw Grid')

            ax = plt.subplot(244)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.imshow(occupancy_grid, cmap='Reds')
            ax.set_title('Occupancy Grid')

            fig.savefig('%s/%d' % (self.viz_dir, len(os.listdir(self.viz_dir))))
            plt.close()

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

    def gridify(self, arr, new_shape):
        shape = (new_shape[0], arr.shape[0] // new_shape[0],
                 new_shape[1], arr.shape[1] // new_shape[1])
        return arr.reshape(shape).mean(-1).mean(1)

    def listen_for_frames(self):
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            if self.save_data:
                np.save('%s/%d.npy' % (self.data_dir, len(os.listdir(self.data_dir))), depth_image)
                cv2.imwrite('%s/%d.png' % (self.data_dir + 'color', len(os.listdir(self.data_dir + 'color'))), color_image)
            self.detect_obstacles_from_above(depth_frame, color_frame)


if __name__ == '__main__':
    try:
        obstacle_detection = ObstacleDetectionNode()
        print('Listening for frames...')
        obstacle_detection.listen_for_frames()
    except rospy.exceptions.ROSInterruptException:
        pass
    finally:
        # Stop streaming
        pipeline.stop()
