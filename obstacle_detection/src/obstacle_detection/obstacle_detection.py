#!/usr/bin/env python3
import os
import glob
import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')  # Fix cv2 import error
import cv2  # TODO Dumb fix please fix
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')  # Fix cv2 import error
import rospy
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2, Imu
import ros_numpy.point_cloud2
import time


class ObstacleDetectionNode:
    def __init__(self):
        self.h, self.w = rospy.get_param('obstacle_detection/realsense/img_h'), \
                         rospy.get_param('obstacle_detection/realsense/img_w')  # realsense depth image size
        self.resolution = rospy.get_param('obstacle_detection/grid_resolution')  # meters per grid cell
        self.grid_size = rospy.get_param('obstacle_detection/grid_size')  # number of rows/cols grid cells
        self.tolerance = rospy.get_param('obstacle_detection/ground_tolerance')  # tolerance in meters above/below ground to ignore
        self.save_imgs = False  # set to True to save local grid visualizations
        self.save_data = False  # set to True to save testing data
        self.localization_topic = rospy.get_param('localization_name')  # filtered global localization topic
        self.viz_dir = 'obstacle_viz/'  # directory to save visualizations
        self.viz_step = 10
        self.viz_i = 0
        self.frame_i = 0  # current frame number
        self.data_dir = 'saved_frames/'  # directory to save testing data

        self.grid_pub = rospy.Publisher('local_occupancy_grid', OccupancyGrid, queue_size=1)

        # RealSense physical orientation in the real world.
        self.CameraPosition = {
            "x": rospy.get_param('obstacle_detection/realsense/x'),  # actual position in meters of RealSense sensor relative to the viewport's center.
            "y": rospy.get_param('obstacle_detection/realsense/y'),  # actual position in meters of RealSense sensor relative to the viewport's center.
            "z": rospy.get_param('obstacle_detection/realsense/z'),  # height in meters of actual RealSense sensor from the floor.
            "roll": 0,  # sensor's roll angle in degrees (these values are with respect to a fixed frame)
            "pitch": -20,  # sensor's pitch angle in degrees.
            "yaw": 0,  # sensor's yaw angle in degrees.
        }

        print('Booting up node...')
        rospy.init_node('obstacle_detection', anonymous=True)

        self.subscribe()

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
        self.last_time = time.time()
        rospy.spin()

    def subscribe(self):
        rospy.Subscriber('realsense_imu_filtered', Imu, self.realsense_callback)
        rospy.Subscriber('realsense/depth/points', PointCloud2, self.receive_point_cloud, buff_size=9830400, queue_size=1)

    def clear_dir(self, dir_name):
        files = glob.glob('%s/*' % (dir_name))
        for f in files:
            try:
                os.remove(f)
            except OSError as e:
                print(e)

    def receive_point_cloud(self, msg):
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        print("recieved valid points: ", xyz.shape[0], time.time()-self.last_time)
        self.last_time = time.time()
        xyz = xyz[::11]  # decimate to save processing power
        self.detect_obstacles_from_above(xyz, None)

    def realsense_callback(self, msg):
        quat = msg.orientation
        euler_angles = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
        print(euler_angles * 57.296)
        self.CameraPosition['roll'] = -(euler_angles[0]) * 57.296
        self.CameraPosition['pitch'] = (euler_angles[1]) * 57.296

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
        rotatePoints(1, 2, self.CameraPosition['pitch']) #rotate on the X&Z plane
        #rotatePoints(0, 1, self.CameraPosition['azimuth']) #rotate on the X&Y

        # Apply offsets for height and linear position of the sensor (from viewport's center)
        pt[:, 2] *= -1  # TODO: Is this still necessary?
        pt[:] += np.float_([self.CameraPosition['x'], self.CameraPosition['y'], self.CameraPosition['z']])
        return pt

    def project_point_cloud_onto_plane(self, xyz_arr, cropping=500, pcnt=0):
        grid_size = 450
        final_grid_size = 450
        proj = xyz_arr[..., [0, 1]]  # take only the X and Y components of point cloud
        proj_img = np.zeros((grid_size, grid_size))
        indices = np.int32(proj * (grid_size / (self.grid_size * self.resolution)))
        try:
            indices[..., 0] += grid_size // 2
            indices = np.clip(indices, 0, grid_size - 1)

            proj_img[indices[..., 0], indices[..., 1]] = 255
            # proj_img = cv2.dilate(proj_img, kernel=np.ones((3, 3)), iterations=1)
            proj_img = cv2.resize(proj_img, (final_grid_size, final_grid_size), interpolation=cv2.INTER_AREA)
            proj_img = np.rot90(proj_img)
        except ValueError:
            pass
        return np.uint8(proj_img)

    def detect_obstacles_from_above(self, xyz_arr, color_frame):
        '''
        Isolate the obstacles in a depth frame and publish a local occupancy grid.
        :param depth_frame: depth frame object from realsense
        :param color_frame: color frame object from realsense
        '''
        self.viz_i += 1
        # convert frame objects to arrays

        xyz_arr[:, [1, 2]] = xyz_arr[:, [2, 1]]  # swap axes to be XYZ since realsense Z is depth
        xyz_arr = self.apply_camera_matrix_orientation(xyz_arr)  # apply height param and orientation from IMU

        if self.save_data:  # save testing data
            # np.save('%s/%d.npy' % (self.data_dir, self.frame_i), depth_image)
            np.save('%s/%d.npy' % (self.data_dir + 'points', self.frame_i), xyz_arr)
            # cv2.imwrite('%s/%d.png' % (self.data_dir + 'color', self.frame_i), color_image)
            self.frame_i += 1

        # rocks are points above the ground plane past the tolerance
        # holes are points below the ground plane past the tolerance
        rocks = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 2] >= self.tolerance])
        holes = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 2] <= -self.tolerance])
        #
        # convert rock projection and hole projection to occupancy grid
        # gridify splits the projection into cells and takes the average number of "occupied" pixels in each cell
        rock_grid = self.gridify(rocks, (self.grid_size, self.grid_size), func=np.mean)
        hole_grid = self.gridify(holes, (self.grid_size, self.grid_size), func=np.mean)
        obs_grid = np.maximum(rock_grid, hole_grid)  # combine rocks and holes into single obstacle grid

        # occupancy_grid = gaussian_filter(obs_grid, sigma=self.kernel_sigma)  # smooth local grid using gaussian kernel
        occupancy_grid = obs_grid / np.max(obs_grid)  # ensure values are 0-1
        occupancy_grid = np.flipud(occupancy_grid)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'  # local grid is in base_link frame

        map_meta_data = MapMetaData()
        map_meta_data.map_load_time = header.stamp
        map_meta_data.resolution = self.resolution
        map_meta_data.width = self.grid_size
        map_meta_data.height = self.grid_size
        map_meta_data.origin = Pose(Point(self.CameraPosition['x'], -self.grid_size * self.resolution / 2, 0),
                                    Quaternion(0, 0, sqrt(2)/2, sqrt(2)/2))  # 90 degree rotation

        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info = map_meta_data
        grid_msg.data = list(np.int8(occupancy_grid.flatten() * 100))  # occupany grid message requires values 0-100

        try:
            self.grid_pub.publish(grid_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass

        if self.save_imgs and self.viz_i % self.viz_step == 0:
            fig = plt.figure(figsize=(20, 10))

            ax = plt.subplot(241)
            ax.set_xticks([])
            ax.set_yticks([])
            # ax.imshow(depth_image, cmap='jet')
            ax.set_title('Depth Frame')

            ax = plt.subplot(245)
            ax.set_xticks([])
            ax.set_yticks([])
            # ax.imshow(color_image)
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
            point_cloud = xyz_arr[::30]
            point_cloud = point_cloud[point_cloud[:, 1] < 4.5, :]
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
