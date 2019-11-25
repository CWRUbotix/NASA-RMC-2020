#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from hci.msg import sensorValue
from autonomy.msg import sensor_value
from autonomy.srv import RobotState, RobotStateResponse

odometry = None
occupancy_grid = None
service = None
sensors = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

sensormap = {
    5  : 0,  # bcAttitudePortPot
    6  : 1,  # bcAttitudeStarboardPot
    9  : 2,  # depLoadCell
    10 : 3,  # excLoadCell
    23 : 4,  # depLowerLimit
    24 : 5,  # depUpperLimit
    25 : 6,  # excForeLimit
    26 : 7, # excAftLimit
    27 : 8, # bcLowerLimit
    28 : 9  # bcUpperLimit
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
    rospy.Subscriber("hci/sensorValue", sensorValue, update_sensors)
    rospy.Subscriber("odometry/filtered_map", Odometry, update_odometry)
    #rospy.Subscriber("", OccupancyGrid, ) #todo

def send_robot_state(req):
    global odometry, occupancy_grid
    (s0, s1, s2, s3, s4, s5, s6, s7, s8, s9) = tuple(sensors)
    sensor_msg = sensor_value(bcAttitudePortPot=s0, bcAttitudeStarboardPot=s1, depLoadCell=s2, excLoadCell=s3, depLowerLimit=s4,
                              depUpperLimit=s5, excForeLimit=s6, excAftLimit=s7, bcLowerLimit=s8, bcUpperLimit=s9)
    return RobotStateResponse(odometry=odometry, grid=occupancy_grid, sensors=sensor_msg)

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




