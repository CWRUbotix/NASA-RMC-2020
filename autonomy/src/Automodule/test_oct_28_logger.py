#!/usr/bin/env python3

import rospy
import sys
import time

from hwctrl.msg import SensorValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R


class robot_state:
    def __init__(self):
        self.gyro0Z = 0.0 #8
        self.gyro1Z = 0.0 #9
        self.acce0X = 0.0 #6
        self.acce1X = 0.0 #7
        self.port_rpm = 0.0 #4
        self.star_rpm = 0.0 #5
        self.x = 0.0 #0
        self.y = 0.0 #1
        self.z = 0.0 #2
        self.theta = 0.0 #3
        self.iter = 0

    def __iter__(self):
        self.iter = 0
        return self

    def __next__(self):
        self.iter += 1
        if self.iter - 1 == 0:
            return self.x
        elif self.iter - 1 == 1:
            return self.y
        elif self.iter - 1 == 2:
            return self.z
        elif self.iter - 1 == 3:
            return self.theta
        elif self.iter - 1 == 4:
            return self.port_rpm
        elif self.iter - 1 == 5:
            return self.star_rpm
        elif self.iter - 1 == 6:
            return self.acce0X
        elif self.iter - 1 == 7:
            return self.acce1X
        elif self.iter - 1 == 8:
            return self.gyro0Z
        elif self.iter - 1 == 9:
            return self.gyro1Z
        else:
            raise StopIteration

currentState = robot_state()
logfile = None

def updateState(msg):
    global currentState

    if msg.sensor_id == 0:
        currentState.port_rpm = msg.value
    elif msg.sensor_id == 1:
        currentState.star_rpm = msg.value
    elif msg.sensor_id == 13:
        currentState.gyro0Z = msg.value
    elif msg.sensor_id == 19:
        currentState.gyro1Z = msg.value
    elif msg.sensor_id == 14:
        currentState.acce0X = msg.value
    elif msg.sensor_id == 20:
        currentState.acce1X = msg.value

def updatePos(msg):
	pos = msg.pose.pose
	currentState.x = pos.position.x
	currentState.y = pos.position.y
	quat = pos.orientation
	theta = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
	currentState.theta = theta[2]

def subscribe():
    rospy.Subscriber('sensorValue', SensorValue, updateState)
    rospy.Subscriber('uwb_nodes', PoseWithCovarianceStamped, updatePos)

def shutdown():
    global logfile
    logfile.close()

def main():
    global logfile
    rospy.init_node("Test_Logger")
    rospy.on_shutdown(shutdown)
    logfile = open(str(sys.path[0]) + '/logs/' +time.strftime("%b-%a-%d-%H-%M-%S.txt"), "w")
    subscribe()

    input("Enter to begin")

    rate = rospy.Rate(2)
    start = time.time()

    while not rospy.is_shutdown():
        log = "time:" + str(time.time() - start)
        tags = ["x_pos:", "y_pos:", "z_pos", "theta:",
                "port_rpm:", "star_rpm:", "acce0X:", "acce1X:", "gyro0Z:", "gyro1Z:"]
        tag = 0
        for sensor in currentState:
            log += tags[tag] + str(sensor) + " "
            tag += 1
        logfile.write(log[:len(log)-1] + "\n")

        rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
