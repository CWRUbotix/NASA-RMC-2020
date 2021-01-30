#!/usr/bin/env python3
import rospy
from math import sqrt, ceil
import numpy as np

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2
import ros_numpy.point_cloud2


class ObstacleDetectionNode:
    def __init__(self):
        self.resolution = rospy.get_param('obstacle_detection/grid_resolution')  # meters per grid cell
        self.grid_size = int(ceil(rospy.get_param('obstacle_detection/grid_size') / self.resolution))  # number of rows/cols grid cells

        self.camera_x_offset = rospy.get_param('obstacle_detection/realsense/x')
        self.camera_y_offset = rospy.get_param('obstacle_detection/realsense/y')

        self.grid_pub = rospy.Publisher('local_occupancy_grid', OccupancyGrid, queue_size=1)

        print('Booting up node...')
        rospy.init_node('obstacle_detection', anonymous=True)

        rospy.Subscriber('input_cloud', PointCloud2, self.receive_point_cloud, queue_size=1)

        rospy.spin()

    def project_point_cloud_onto_plane(self, xyz_arr):
        bin_range = [[-0.5 * self.grid_size * self.resolution, 0.5 * self.grid_size * self.resolution],
                     [0, self.grid_size * self.resolution]]

        grid, _, _ = np.histogram2d(xyz_arr[..., 0], xyz_arr[..., 1], bins=self.grid_size, range=bin_range)

        # Histogram has x in vertical direction (rows) so transpose
        return grid.T

    def receive_point_cloud(self, msg):
        '''
        Processes the pointcloud into a costmap
        '''
        xyz_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

        # Flatten the points onto a grid
        obs_grid = self.project_point_cloud_onto_plane(xyz_arr)

        obs_grid[obs_grid != 0] = 1

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'base_link'  # local grid is in base_link frame

        map_meta_data = MapMetaData()
        map_meta_data.map_load_time = header.stamp
        map_meta_data.resolution = self.resolution
        map_meta_data.width = self.grid_size
        map_meta_data.height = self.grid_size
        map_meta_data.origin = Pose(Point(self.camera_x_offset, self.grid_size * self.resolution / 2 + self.camera_y_offset, 0),
                                    Quaternion(0, 0, -sqrt(2)/2, sqrt(2)/2))  # 90 degree rotation

        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info = map_meta_data
        grid_msg.data = list(np.int8(obs_grid.flatten() * 100))  # occupany grid message requires values 0-100

        try:
            self.grid_pub.publish(grid_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass
