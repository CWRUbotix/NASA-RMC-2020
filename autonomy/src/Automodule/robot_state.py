#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import hci.msg as hci
import autonomy.msg as aut
from autonomy.srv import RobotState, RobotStateResponse

odometry = None
occupancy_grid = None
service = None
sensors = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

sensormap = {
    2  : 0,  # depWinchEncoder
    3  : 1,  # bcBeltEncoder
    4  : 2,  # bcTranslationEncoder
    5  : 3,  # bcAttitudePortPot
    6  : 4,  # bcAttitudeStarboardPot
    9  : 5,  # depLoadCell
    10 : 6,  # excLoadCell
    23 : 7,  # depLowerLimit
    24 : 8,  # depUpperLimit
    25 : 9,  # excForeLimit
    26 : 10, # excAftLimit
    27 : 11, # bcLowerLimit
    28 : 12  # bcUpperLimit
}

def update_odometry(msg):
    global odometry
    odometry = msg

def update_occupancy_grid(msg):
    global occupancy_grid
    occupancy_grid = msg

def update_sensors(msg):
    global sensors, sensormap
    if msg.sensorID in sensormap.keys():
        sensors[sensormap[msg.sensorID]] = msg.value

def subscribe():
    rospy.Subscriber("hci/sensorValue", hci.sensorValue, update_sensors)
    rospy.Subscriber("odometry/filtered_map", Odometry, update_odometry)
    #rospy.Subscriber("", OccupancyGrid, ) #todo

def send_robot_state(req):
    global odometry, occupancy_grid
    (s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12) = tuple(sensors)
    sensor_msg = aut.sensorValue(s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12)
    return RobotStateResponse(time=rospy.get_rostime(), odometry=odometry, grid=occupancy_grid, sensors=sensor_msg)

def on_shut_down():
    global service
    service.shutdown("robot_state no longer running")

def main():
    global service
    rospy.init_node("robot_state")
    subscribe()
    service = rospy.Service("robot_state", RobotState, send_robot_state)
    rospy.spin()




if __name__ == "__main__":
    main()




