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


class ObstacleDetectionNode:

    def __init__(self):
        self.h, self.w = rospy.get_param('obstacle_detection/realsense/img_h'), \
                         rospy.get_param('obstacle_detection/realsense/img_w')  # realsense depth image size
        self.resolution = rospy.get_param('obstacle_detection/grid_resolution')  # meters per grid cell
        self.grid_size = rospy.get_param('obstacle_detection/grid_size')  # number of rows/cols grid cells
        self.tolerance = rospy.get_param('obstacle_detection/ground_tolerance')  # tolerance in meters above/below ground to ignore
        self.kernel_sigma = 0.5  # standard deviation of gaussian kernel used to smooth local grid
        self.save_imgs = True  # set to True to save local grid visualizations
        self.save_data = True  # set to True to save testing data
        self.localization_topic = rospy.get_param('localization_name')  # filtered global localization topic
        self.viz_dir = 'obstacle_viz/'  # directory to save visualizations
        self.viz_step = 20
        self.viz_i = 0
        self.frame_i = 0  # current frame number
        self.data_dir = 'saved_frames/'  # directory to save testing data

        # RealSense physical orientation in the real world.
        self.CameraPosition = {
            "x":rospy.get_param('obstacle_detection/realsense/x'),  # actual position in meters of RealSense sensor relative to the viewport's center.
            "y": rospy.get_param('obstacle_detection/realsense/y'),  # actual position in meters of RealSense sensor relative to the viewport's center.
            "z": rospy.get_param('obstacle_detection/realsense/z'),  # height in meters of actual RealSense sensor from the floor.
            "roll": 0,  # sensor's roll angle in degrees (trig function for this is commented out by default).
            "azimuth": 0,  # sensor's yaw angle in degrees.
            "elevation": 20,  # sensor's pitch angle in degrees.
        }

        print('Booting up node...')
        rospy.init_node('obstacleDetection', anonymous=True)

        os.makedirs(self.viz_dir, exist_ok=True)
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.data_dir + 'color', exist_ok=True)
        os.makedirs(self.data_dir + 'localization', exist_ok=True)
        os.makedirs(self.data_dir + 'points', exist_ok=True)

        self.clear_dir(self.viz_dir)
        self.clear_dir(self.data_dir)
        self.clear_dir(self.data_dir + 'color')
        self.clear_dir(self.data_dir + 'localization')
        self.clear_dir(self.data_dir + 'points')

    def clear_dir(self, dir_name):
        files = glob.glob('%s/*' % (dir_name))
        for f in files:
            try:
                os.remove(f)
            except OSError as e:
                print(e)

    def realsense_callback(self, msg):
        self.CameraPosition['roll'] = -(msg.x) * 57.296
        self.CameraPosition['elevation'] = -(msg.z + math.pi / 2) * 57.296

    def apply_camera_matrix_orientation(self, pt):
        """
        Transforms the entire 3D point cloud according to the position of the Kinect.  Basically an efficient vectorized version of ``apply_camera_orientation``
        Args:
            pt (:obj:`numpy.float32`) : 3D point cloud represented by an [N, 3] Numpy array
            CameraPosition (:obj:`dict`) : the current position and orientation of the Kinect sensor.  Either hardcoded in the ``CameraPosition`` dictionary, or computed from the best-fit ground plane
        Returns:
            Updated point cloud represented as an [N, 3] numpy array in correct orientation to the Kinect
        """
        # bacically this is a vectorized version of applyCameraOrientation()
        # uses same trig to rotate a vertex around a gimbal.
        def rotatePoints(ax1, ax2, deg):
            # math to rotate vertexes around a center point on a plane.
            hyp = np.sqrt(pt[:, ax1] ** 2 + pt[:, ax2] ** 2) # Get the length of the hypotenuse of the real-world coordinate from center of rotation, this is the radius!
            d_tan = np.arctan2(pt[:, ax2], pt[:, ax1]) # Calculate the vertexes current angle (returns radians that go from -180 to 180)

            cur_angle = np.degrees(d_tan) % 360 # Convert radians to degrees and use modulo to adjust range from 0 to 360.
            new_angle = np.radians((cur_angle + deg) % 360) # The new angle (in radians) of the vertexes after being rotated by the value of deg.

            pt[:, ax1] = hyp * np.cos(new_angle) # Calculate the rotated coordinate for this axis.
            pt[:, ax2] = hyp * np.sin(new_angle) # Calculate the rotated coordinate for this axis.

        rotatePoints(0, 2, self.CameraPosition['roll']) #rotate on the Y&Z plane # Disabled because most tripods don't roll. If an Inertial Nav Unit is available this could be used)
        rotatePoints(1, 2, self.CameraPosition['elevation']) #rotate on the X&Z plane
        #rotatePoints(0, 1, self.CameraPosition['azimuth']) #rotate on the X&Y

        # Apply offsets for height and linear position of the sensor (from viewport's center)
        pt[:, 2] *= -1  # TODO: Is this still necessary?
        pt[:] += np.float_([self.CameraPosition['x'], self.CameraPosition['y'], self.CameraPosition['z']])
        return pt

    def project_point_cloud_onto_plane(self, xyz_arr, resize_factor=10, cropping=500, pcnt=0):
        grid_size = 4500
        proj = xyz_arr[..., [0, 1]]  # take only the X and Y components of point cloud
        proj_img = np.zeros((grid_size, grid_size))
        indices = np.int32(proj * 1000)
        try:
            indices[..., 0] += grid_size // 2
            indices = np.clip(indices, 0, grid_size - 1)
            proj_img[indices[..., 0], indices[..., 1]] = 255
            new_size = grid_size // resize_factor
            proj_img = cv2.rotate(proj_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            #_, proj_img = cv2.threshold(proj_img, 127, 255, cv2.THRESH_BINARY)
            #proj_img = cv2.dilate(proj_img, kernel=np.ones((3, 3)), iterations=1)
            proj_img = cv2.resize(proj_img, (new_size, new_size), interpolation=cv2.INTER_LINEAR)
        except ValueError:
            pass
        return np.uint8(proj_img)

    def detect_obstacles_from_above(self, depth_frame, color_frame):
        '''
        Isolate the obstacles in a depth frame and publish a local occupancy grid.
        :param depth_frame: depth frame object from realsense
        :param color_frame: color frame object from realsense
        '''
        self.viz_i += 1
        # convert frame objects to arrays
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
        xyz_arr[:, [1, 2]] = xyz_arr[:, [2, 1]]  # swap axes to be XYZ since realsense Z is depth
        xyz_arr = self.apply_camera_matrix_orientation(xyz_arr)  # apply height param and orientation from IMU


        if self.save_data:  # save testing data
            np.save('%s/%d.npy' % (self.data_dir, self.frame_i), depth_image)
            np.save('%s/%d.npy' % (self.data_dir + 'points', self.frame_i), xyz_arr)
            cv2.imwrite('%s/%d.png' % (self.data_dir + 'color', self.frame_i), color_image)
            self.frame_i += 1

        # rocks are points above the ground plane past the tolerance
        # holes are points below the ground plane past the tolerance
        rocks = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 2] >= self.tolerance])
        holes = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 2] <= -self.tolerance])

        # convert rock projection and hole projection to occupancy grid
        # gridify splits the projection into cells and takes the average number of "occupied" pixels in each cell
        rock_grid = self.gridify(rocks, (self.grid_size, self.grid_size))
        hole_grid = self.gridify(holes, (self.grid_size, self.grid_size))
        obs_grid = np.maximum(rock_grid, hole_grid)  # combine rocks and holes into single obstacle grid

        occupancy_grid = gaussian_filter(obs_grid, sigma=self.kernel_sigma)  # smooth local grid using gaussian kernel
        occupancy_grid = occupancy_grid / np.max(occupancy_grid)  # ensure values are 0-1

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom'  # local grid is in odom frame

        map_meta_data = MapMetaData()
        map_meta_data.map_load_time = rospy.Time.now()
        map_meta_data.resolution = self.resolution
        map_meta_data.width = self.grid_size
        map_meta_data.height = self.grid_size
        map_meta_data.origin = Pose(Point(0, 0, 0),
                                    Quaternion(0, 0, 0, 1))

        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info = map_meta_data
        grid_msg.data = list(np.int8(occupancy_grid.flatten() * 100))  # occupany grid message requires values 0-100

        try:
            pub = rospy.Publisher('local_occupancy_grid', OccupancyGrid, queue_size=1)
            pub.publish(grid_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass

        if self.save_imgs and self.viz_i % self.viz_step == 0:
            fig = plt.figure(figsize=(20, 10))

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
            ax.imshow(np.maximum(rocks, holes))
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

            ax = plt.subplot(246, projection='3d')
            point_cloud = xyz_arr[::150]
            point_cloud = point_cloud[point_cloud[: , 1] < 4.5, :]
            ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], s=1)
            ax.set_xlim(-2.5, 2.5)
            ax.set_ylim(0, 5)
            ax.set_zlim(-2.5, 2.5)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            ax = plt.subplot(247)
            ax.scatter(point_cloud[:, 1], point_cloud[:, 2], s=1)
            ax.plot([0, 5], [self.tolerance, self.tolerance])
            ax.plot([0, 5], [-self.tolerance, -self.tolerance])
            ax.set_xlim(0, 5)
            ax.set_ylim(-1, 1)
            ax.set_xlabel('Y')
            ax.set_ylabel('Z')

            ax = plt.subplot(248)
            ax.scatter(point_cloud[:, 0], point_cloud[:, 2], s=1)
            ax.set_xlim(-2.5, 2.5)
            ax.set_ylim(-1, 1)
            ax.set_xlabel('X')
            ax.set_ylabel('Z')

            plt.tight_layout()
            fig.savefig('%s/%d' % (self.viz_dir, len(os.listdir(self.viz_dir))))
            plt.close()

    @staticmethod
    def gridify(arr, new_shape, func=np.mean):
        '''
        Divides a 2D array into a grid where each cell takes on the value
        of func over the entries within the cell
        :param arr: 2D array of "occupied" pixels
        :param new_shape: (rows, cols) of returned grid
        :param func: aggregate function to use to assign each grid cell a value, default is mean
        :return: 2D array of shape new_shape
        '''
        shape = (new_shape[0], arr.shape[0] // new_shape[0],
                 new_shape[1], arr.shape[1] // new_shape[1])
        return func(func(arr.reshape(shape), axis=-1), axis=1)

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


if __name__ == '__main__':
    try:
        obstacle_detection = ObstacleDetectionNode()
        print('Listening for frames...')
        rospy.Subscriber('realsense_orientation', Vector3, obstacle_detection.realsense_callback)
        obstacle_detection.listen_for_frames()
    except rospy.exceptions.ROSInterruptException:
        pass
    finally:
        # Stop streaming
        pipeline.stop()
