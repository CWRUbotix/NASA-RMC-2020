#!/usr/bin/env python3
import os
import sys
import glob
import math
import rospy
import math
import matplotlib
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
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
        device = fn.openDevice(serial, pipeline=pipeline)

        self.listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        device.setColorFrameListener(listener)
        device.setIrAndDepthFrameListener(listener)

        device.start()

        # NOTE: must be called after device.start()
        self.registration = Registration(device.getIrCameraParams(),
                                    device.getColorCameraParams())

        
        self.focal_x = device.getIrCameraParams().fx  # focal length x
        self.focal_y = device.getIrCameraParams().fy  # focal length y
        self.principal_x = device.getIrCameraParams().cx  # principal point x
        self.principal_y = device.getIrCameraParams().cy  # principal point y
        self.undistorted = Frame(h, w, 4)
        self.registered = Frame(h, w, 4)

    def listen_for_frames(self):
        while not rospy.is_shutdown():
            frames = self.listener.waitForNewFrame()
            depth_frame = frames["depth"]
            color = frames["color"]
            self.registration.apply(color, depth_frame, self.undistorted, self.egistered)
            color_frame = self.registered.asarray(np.uint8)

            img = depth_frame.asarray(np.float32)
            img = cv2.flip(img, 1)

            # TODO: Detect obstacles

            listener.release(frames)

        listener.release(frames)
        device.stop()
        device.close()

        sys.exit(0)



if __name__ == '__main__':
    obstacle_detection = ObstacleDetectionNode()

