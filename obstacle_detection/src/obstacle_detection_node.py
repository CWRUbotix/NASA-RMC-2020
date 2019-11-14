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
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry

from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
try:
    from pylibfreenect2 import OpenCLPacketPipeline
    pipeline = OpenCLPacketPipeline()
except:
    from pylibfreenect2 import CpuPacketPipeline
    pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)


class ObstacleDetectionNode:

    def __init__(self):
        self.h, self.w = 512, 424
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
        
        self.start_kinect()
        print('Booting up node...')
        rospy.init_node('obstacleDetection', anonymous=True)

        os.makedirs(self.viz_dir, exist_ok=True)
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.data_dir + '_color', exist_ok=True)
        os.makedirs(self.data_dir + '_localization', exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

    def start_kinect(self):
        # Create and set logger
        logger = createConsoleLogger(LoggerLevel.Debug)
        setGlobalLogger(None)

        fn = Freenect2()
        num_devices = fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        serial = fn.getDeviceSerialNumber(0)
        self.device = fn.openDevice(serial, pipeline=pipeline)

        self.listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        self.device.setColorFrameListener(self.listener)
        self.device.setIrAndDepthFrameListener(self.listener)

        self.device.start()

        # NOTE: must be called after device.start()
        self.registration = Registration(self.device.getIrCameraParams(),
                                    self.device.getColorCameraParams())

        
        self.focal_x = self.device.getIrCameraParams().fx  # focal length x
        self.focal_y = self.device.getIrCameraParams().fy  # focal length y
        self.principal_x = self.device.getIrCameraParams().cx  # principal point x
        self.principal_y = self.device.getIrCameraParams().cy  # principal point y
        self.undistorted = Frame(self.h, self.w, 4)
        self.registered = Frame(self.h, self.w, 4)

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
        print(xyz_arr[..., 2].min(), xyz_arr[..., 2].max())
        z_projection = self.project_point_cloud_onto_plane(xyz_arr)
        z_projection_thresh = self.project_point_cloud_onto_plane(xyz_arr[xyz_arr[..., 2] > -0.2])
        cm = plt.get_cmap('viridis')
        # Apply the colormap like a function to any array:
        color = cm(z_projection)
        color = np.uint8(color[:, :, :3] * 255)
        ret, thresh = cv2.threshold(z_projection_thresh, 16, 255, cv2.THRESH_BINARY)

        # begin contour detection
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cntr in contours:
            try:
                # calculate diameter of equivalent circle
                # this measurement is only used for checking if countours fit our bounds
                area = cv2.contourArea(cntr)
                equi_diameter = np.sqrt(4 * area / np.pi)
                # Hardcoded Diameter Range in pixels
                LOW_DIAMETER_BOUND = 10
                HIGH_DIAMETER_BOUND = 50
                x, y, obj_length, obj_height = cv2.boundingRect(cntr)
                if obj_length > LOW_DIAMETER_BOUND and obj_length < HIGH_DIAMETER_BOUND and obj_height > LOW_DIAMETER_BOUND and obj_height < HIGH_DIAMETER_BOUND:
                    moment = cv2.moments(cntr)  # get the centroid of the obstacle using its moment
                    cx = int(moment['m10'] / moment['m00'])
                    cy = int(moment['m01'] / moment['m00'])
                    cv2.rectangle(color, (x, y), (x + obj_length, y + obj_height), (0, 255, 0), 2)

            except cv2.error as e:
                print(e)
                pass

        if self.save_imgs:
            fig = plt.figure(figsize=(15, 5))

            ax = plt.subplot(131)
            ax.imshow(color)
            ax = plt.subplot(132)
            ax.imshow(z_projection, cmap='viridis')
            ax = plt.subplot(133)
            ax.imshow(z_projection_thresh, cmap='viridis')

            fig.savefig('%s/%d' % (self.viz_dir, len(os.listdir('saved/'))))
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

    def listen_for_frames(self):
        while not rospy.is_shutdown():
            print('waiting for frame...')
            frames = self.listener.waitForNewFrame()
            print('new frame...')
            depth_frame = frames["depth"]
            color = frames["color"]

            self.registration.apply(color, depth_frame, self.undistorted, self.registered)
            color_frame = self.registered.asarray(np.uint8)
            if self.save_data:
                np.save('%s/%d.npy' % (self.data_dir, len(os.listdir(self.data_dir))), depth_frame)
                np.save('%s/%d.npy' % (self.data_dir + '_color', len(os.listdir(self.data_dir + '_color'))), color_frame)
                np.save('%s/%d.npy' % (self.data_dir + '_localization', len(os.listdir(self.data_dir + '_localization'))), np.array(self.robot_x[-1], self.robot_y[-1], self.robot_pitch[-1]))


            img = depth_frame.asarray(np.float32)
            img = cv2.flip(img, 1)
            # TODO: Detect obstacles
            self.detect_obstacles_from_above(img)

            self.listener.release(frames)

        self.listener.release(frames)
        self.device.stop()
        self.device.close()

        sys.exit(0)


if __name__ == '__main__':
    obstacle_detection = ObstacleDetectionNode()
    print('Listening for frames...')
    sub = rospy.Subscriber(obstacle_detection.localization_topic, Odometry, obstacle_detection.localization_listener, queue_size=1)
    rospy.spin()
    obstacle_detection.listen_for_frames()


