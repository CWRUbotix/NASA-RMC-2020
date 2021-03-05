#!/usr/bin/env python3
import rospy
from glenn_utilities.led_visualizer import LEDVisualizer

if __name__ == '__main__':
    try:
        led_visualizer = LEDVisualizer()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
