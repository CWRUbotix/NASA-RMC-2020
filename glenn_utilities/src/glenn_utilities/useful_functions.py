#!/usr/bin/env python3
import math
from geometry_msgs.msg import Quaternion


def wrap_angle(x):
    """Convert angles to be within -pi to pi"""
    return (x + 3.14159265359) % 6.28318530718 - 3.14159265359


def euclidian_distance(x1, y1, x2, y2):
    """Pythagorean theorem"""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def euclidian_distance(x, y):
    """Pythagorean theorem"""
    return math.sqrt(x ** 2 + y ** 2)


def quaternion_to_yaw(quat):
    """
    Converts a "flat" (confined to 2D plane)
    quaternion to the yaw angle
    """
    return 2.0 * math.arctan2(quat.z, quat.w)


def yaw_to_quaternion(yaw):
    """Conerts a yaw angle to a flat quaternion"""
    return Quaternion(x=.0, y=0.0, z=math.sin(0.5 * yaw),
                      w=math.cos(0.5 * yaw))
