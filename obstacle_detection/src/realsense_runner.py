#!/usr/bin/env python3
import os
import sys
import glob
import cv2
import math
import rospy
import math
import matplotlib
import numpy as np
import pandas as pd
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import signal
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import gaussian_filter

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

from realsense_utils import *
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth,
                     rospy.get_param('obstacle_detection/realsense/img_h'),
                     rospy.get_param('obstacle_detection/realsense/img_w'),
                     rs.format.z16,
                     rospy.get_param('obstacle_detection/realsense/fps'))
config.enable_stream(rs.stream.color,
                     rospy.get_param('obstacle_detection/realsense/img_h'),
                     rospy.get_param('obstacle_detection/realsense/img_w'),
                     rs.format.bgr8,
                     rospy.get_param('obstacle_detection/realsense/fps'))

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()

class RealsenseRunnerNode:
    def __init__(self):
        pass

    def listen_for_frames(self):
        '''
        While the node is running, wait for color and depth frames from the RealSense and publish a local occupancy
        grid of the obstacles detected in each frame
        '''
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            self.detect_obstacles_from_above(depth_frame, color_frame)

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = np.asanyarray(
                colorizer.colorize(depth_frame).get_data())

            if state.color:
                mapped_frame, color_source = color_frame, color_image
            else:
                mapped_frame, color_source = depth_frame, depth_colormap

            # compute point cloud based on realsense intrinsics
            points = pc.calculate(depth_frame)
            pc.map_to(mapped_frame)

            # pointcloud data to arrays
            v, t = points.get_vertices(), points.get_texture_coordinates()
            xyz_arr = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # (n x 3) XZY array