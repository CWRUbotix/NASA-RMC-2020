#!/usr/bin/env python2

import math

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy import ndimage

import rospy
import tf2_ros, tf2_geometry_msgs, tf2_sensor_msgs
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion
from sensor_msgs.msg import PointCloud2
import ros_numpy.point_cloud2


class GlobalOccupancyGrid:

    def __init__(self):
        rospy.init_node('global_map', anonymous=True)

        self.local_grid = None
        self.arena_length = rospy.get_param('arena_y')
        self.arena_width = rospy.get_param('arena_x')
        self.resolution = rospy.get_param('obstacle_detection/grid_resolution')
        self.map_buffer = int(math.ceil(rospy.get_param('obstacle_detection/map_buffer') / self.resolution))
        self.camera_offset = [rospy.get_param('obstacle_detection/realsense/x'),
                              rospy.get_param('obstacle_detection/realsense/y'),
                              rospy.get_param('obstacle_detection/realsense/yaw')]
        self.global_grid_shape = (int(self.arena_length / self.resolution) + 2 * self.map_buffer,
                                  int(self.arena_width / self.resolution) + 2 * self.map_buffer)
        
        self.global_grid = np.ones(self.global_grid_shape) * 0.2  # Initial probability

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        self.grid_pub = rospy.Publisher("global_occupancy_grid", OccupancyGrid, queue_size=1)
        # rospy.Subscriber("local_occupancy_grid", OccupancyGrid, self.local_grid_callback, queue_size=1)
        rospy.Subscriber("input_cloud", PointCloud2, self.receive_point_cloud, queue_size=1)


    def receive_point_cloud(self, msg):

        try:
            msg = self.tf_buffer.transform(msg, "map", rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn(e)
            return

        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

        bin_range = [[-self.map_buffer * self.resolution, self.arena_width + self.map_buffer * self.resolution],
                     [-self.map_buffer * self.resolution, self.arena_length + self.map_buffer * self.resolution]]

        grid, _, _ = np.histogram2d(points[..., 0], points[..., 1], bins=self.global_grid_shape[::-1], range=bin_range)

        # Histogram has x in vertical direction (rows) so transpose
        grid = grid.T
        grid[grid != 0] = 1

        fp_rate = 0.05
        fn_rate = 0.5
        roi = (grid == 1)

        # Use bayes theorem on detecting an obstacle
        self.global_grid[roi] = self.global_grid[roi] * (1 - fn_rate) / (self.global_grid[roi] * (1 - fn_rate) + (1 - self.global_grid[roi]) * fp_rate)

        # Use bayes theorem on not detecting an obstacle
        self.global_grid[~roi] = self.global_grid[~roi] * fn_rate / ((1 - self.global_grid[~roi]) * (1 - fp_rate) + self.global_grid[~roi] * fn_rate)

        self.global_grid = np.clip(self.global_grid, 0.01, 0.9999999)
        # try:
        #     grid_origin = self.tf_buffer.transform(grid_origin, "map", rospy.Duration(0.1))
        #     rot = self.tf_buffer.lookup_transform("base_link", "map", msg.header.stamp, rospy.Duration(0.1)).transform.rotation
        # except Exception as e:
        #     rospy.logwarn(e)
        #     return

        # grid_origin = np.array([grid_origin.point.x, grid_origin.point.y])
        # robot_angle = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('zyx')[0]

        # Camera has ~90 FOV. Set cells outside of that range to 'unknown'
        # indices = np.indices(self.local_grid.shape)
        # mask = abs(indices[1] - self.local_grid.shape[1] / 2) < (self.local_grid.shape[0] - indices[0])
        # mask = np.reshape(mask, (grid_size, grid_size))
        # self.local_grid[~mask] = -1

        #  Rotate image using degrees for some dumb reason
        # rotated_grid = ndimage.rotate(self.local_grid, (-robot_angle * 180 / np.pi + self.camera_offset[2]),
        #                               mode='constant', cval=-1, reshape=True)  # rotate grid by camera + robot angle

        # convert to grid indices:
        # grid_origin = grid_origin / self.resolution

        # flip due to coordinate system of matrix being top left
        # Switch local origin from center of image to top left based on new width of rotated image
        # grid_origin = [int(np.round(self.global_grid_shape[0] - grid_origin[1] - rotated_grid.shape[0] / 2)),
        #                 int(np.round(grid_origin[0] - rotated_grid.shape[1] / 2))]

        # Account for map origin not being the 0,0 cell


        # counts[~roi] = 0

        # global_additions = self.paste(self.global_totals, rotated_grid, grid_origin)
        # window_size = 100
        # self.global_totals[roi] = (self.global_totals[roi] * np.minimum(self.global_counts[roi], window_size) + global_additions[roi]) \
        #                           / (np.minimum(self.global_counts[roi], window_size) + 1)

        # self.global_counts += counts

        # self.global_grid = np.divide(self.global_totals, self.global_counts, out=np.zeros_like(self.global_totals), where=self.global_counts!=0)
        # self.global_grid = np.nan_to_num(self.global_grid, copy=False)
        # self.global_grid /= np.max(self.global_grid)  # ensure values are 0-1

        # self.global_grid[self.global_grid >= 50] = 100
        # self.global_grid[self.global_totals >= 50] = 150 + self.global_totals[self.global_totals >= 50]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        map_meta_data = MapMetaData()
        map_meta_data.map_load_time = rospy.Time.now()
        map_meta_data.resolution = self.resolution

        # x is coloumns, not rows
        map_meta_data.width = self.global_grid_shape[1]
        map_meta_data.height = self.global_grid_shape[0]

        map_meta_data.origin = Pose(Point(-self.map_buffer * self.resolution, -self.map_buffer * self.resolution, 0),
                                    Quaternion(0, 0, 0, 1))

        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info = map_meta_data
        grid_msg.data = list((np.int8((self.global_grid * 100).flatten())))

        try:
            self.grid_pub.publish(grid_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass
