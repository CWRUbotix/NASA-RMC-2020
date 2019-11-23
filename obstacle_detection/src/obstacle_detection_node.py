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

from nav_msgs.msg import Odometry

from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)
# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(logger)

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()

# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)
focal_x = device.getIrCameraParams().fx  # focal length x
focal_y = device.getIrCameraParams().fy  # focal length y
principal_x = device.getIrCameraParams().cx  # principal point x
principal_y = device.getIrCameraParams().cy  # principal point y


class ObstacleDetectionNode:

    def __init__(self):
        self.h, self.w = 512, 424
        self.ground_plane_height = -0.23
        self.grid_size = 50
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
        #camera information based on the Kinect v2 hardware
        self.CameraParams = {
          "cx":254.878,
          "cy":205.395,
          "fx":365.456,
          "fy":365.456,
          "k1":0.0905474,
          "k2":-0.26819,
          "k3":0.0950862,
          "p1":0.0,
          "p2":0.0,
        }
        # Kinect's physical orientation in the real world.
        self.CameraPosition = {
            "x": 0, # actual position in meters of kinect sensor relative to the viewport's center.
            "y": 0, # actual position in meters of kinect sensor relative to the viewport's center.
            "z": 0, # height in meters of actual kinect sensor from the floor.
            "roll": 0, # angle in degrees of sensor's roll (used for INU input - trig function for this is commented out by default).
            "azimuth": 0, # sensor's yaw angle in degrees.
            "elevation": -30, # sensor's pitch angle in degrees.
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
            files = glob.glob('%s/*' % self.data_dir + 'color')
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
        

    def project_point_cloud_onto_plane(self, xyz_arr, resize_factor=10, cropping=500):
        print(xyz_arr.shape)
        normal = np.array([[1, 0, 0], [0, 1, 0]])
        proj = np.matmul(xyz_arr, normal.T)
        proj_img = np.zeros((4500, 4500))
        indices = np.int32(proj * 1000)
        indices[..., 1] += 4500 // 2
        indices = np.clip(indices, 0, 4499)
        proj_img[indices[..., 0], indices[..., 1]] = 255
        proj_img = proj_img[cropping:4500 - cropping, cropping:4500 - cropping]
        new_size = 4500 // resize_factor - cropping // resize_factor
        proj_img = cv2.resize(proj_img, (new_size, new_size), interpolation=cv2.INTER_AREA)
        proj_img = cv2.dilate(proj_img, np.ones((3, 3)), iterations=2)
        proj_img = cv2.blur(proj_img, (5, 5))
        return np.uint8(proj_img)

    def depth_matrix_to_point_cloud(self, z, scale=1000):
        C, R = np.indices(z.shape)

        R = np.subtract(R, self.CameraParams['cx'])
        R = np.multiply(R, z)
        R = np.divide(R, self.CameraParams['fx'] * scale)

        C = np.subtract(C, self.CameraParams['cy'])
        C = np.multiply(C, z)
        C = np.divide(C, self.CameraParams['fy'] * scale)

        return np.column_stack((z.ravel() / scale, R.ravel(), -C.ravel()))

    def detect_obstacles_from_above(self, frame):
        frame[frame < 1000] = 0
        xyz_arr = self.depth_matrix_to_point_cloud(frame)
        # point_height = xyz_arr[]
        print(xyz_arr[..., 2].mean())
        rocks = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 2] >= self.ground_plane_height + self.tolerance])
        holes = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 2] < self.ground_plane_height - self.tolerance])
        rock_grid = self.gridify(rocks, (self.grid_size, self.grid_size))
        hole_grid = self.gridify(holes, (self.grid_size, self.grid_size))
        obs_grid = np.maximum(rock_grid, hole_grid)
        kernel = np.ones((self.kernel_size, self.kernel_size))

        occupancy_grid = signal.convolve2d(obs_grid, kernel, boundary='symm', mode='same')

        if self.save_imgs:
            fig = plt.figure(figsize=(15, 5))

            ax = plt.subplot(131)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.imshow(frame / 4500., cmap='Reds')
            ax.set_title('Depth Frame')

            ax = plt.subplot(132)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.imshow(obs_grid, cmap='Reds')
            ax.set_title('Raw Grid')

            ax = plt.subplot(133)
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
            print('waiting for frame...')
            frames = listener.waitForNewFrame()
            print('new frame...')
            depth_frame = frames["depth"]
            color = frames["color"]

            registration.apply(color, depth_frame, undistorted, registered)
            color_frame = registered.asarray(np.uint8)
            if self.save_data:
                np.save('%s/%d.npy' % (self.data_dir, len(os.listdir(self.data_dir))), depth_frame.asarray())
                cv2.imwrite('%s/%d.png' % (self.data_dir + 'color', len(os.listdir(self.data_dir + 'color'))), color.asarray())
                #np.save('%s/%d.npy' % (self.data_dir + 'color', len(os.listdir(self.data_dir + 'color'))), color_frame)
                #np.save('%s/%d.npy' % (self.data_dir + 'localization', len(os.listdir(self.data_dir + 'localization'))), np.array(self.robot_x[-1], self.robot_y[-1], self.robot_pitch[-1]))


            img = depth_frame.asarray(np.float32)
            img = cv2.flip(img, 1)
            # TODO: Detect obstacles
            self.detect_obstacles_from_above(img)

            listener.release(frames)

        listener.release(frames)
        device.stop()
        device.close()

        sys.exit(0)


if __name__ == '__main__':
	try:
	    obstacle_detection = ObstacleDetectionNode()
	    print('Listening for frames...')
	    #sub = rospy.Subscriber(obstacle_detection.localization_topic, Odometry, obstacle_detection.localization_listener, queue_size=1)
	    #rospy.spin()
	    obstacle_detection.listen_for_frames()
	except rospy.exceptions.ROSInterruptException:
		pass
