#!/usr/bin/env python3
import cv2
import numpy as np

import rospy
from hwctrl.msg import CanFrame

class LEDVisualizer:
    def __init__(self):
        rospy.init_node("led_visualizer")

        rospy.Subscriber("can_frames_tx", CanFrame, self.receive_can_frames)


    def receive_can_frames(self, msg):
        # BGR not RGB
        color = np.array([msg.data[2], msg.data[1], msg.data[0]], dtype=np.uint8)
        # Image size = 200
        image = np.tile(color, [200, 200, 1])

        cv2.imshow("", image)
        cv2.waitKey(50)
