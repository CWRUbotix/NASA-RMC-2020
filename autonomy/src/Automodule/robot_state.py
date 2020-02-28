#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from hci.msg import sensorValue
from hwctrl.msg import MotorData
from autonomy.msg import sensor_value
from autonomy.srv import RobotState, RobotStateResponse

odometry = None
occupancy_grid = None
service = None
sensors = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

sensormap = {
    0  : 0,  # port encoder
    1  : 1,  # starboard encoder
    5  : 2,  # bcAttitudePortPot
    6  : 3,  # bcAttitudeStarboardPot
    9  : 4,  # depLoadCell
    10 : 5,  # excLoadCell
    23 : 6,  # depLowerLimit
    24 : 7,  # depUpperLimit
    25 : 8,  # excForeLimit
    26 : 9, # excAftLimit
    27 : 10, # bcLowerLimit
    28 : 11  # bcUpperLimit
}

def update_odometry(msg):
    global odometry
    odometry = msg

def update_occupancy_grid(msg):
    global occupancy_grid
    occupancy_grid = msg

def update_sensors(msg):
    global sensors, sensormap
    if msg.sensorID in sensormap.keys() and msg.id != 0 and msg.id != 1:
        sensors[sensormap[msg.sensorID]] = msg.value

def update_encoders(msg):
    global sensors, sensormap
    if msg.id in sensormap.keys():
        if msg.id == 0 and msg.data_type == 0:  # port message and RPM value
            sensors[sensormap[msg.id]] = msg.value
        if msg.id == 1 and msg.data_type == 0:  # starboard message and RPM value
            sensors[sensormap[msg.id]] = msg.value



def subscribe():
    rospy.Subscriber("hci/sensorValue", sensorValue, update_sensors)
    rospy.Subscriber("motor_data", MotorData, update_encoders)
    rospy.Subscriber("odometry/filtered_map", Odometry, update_odometry)
    rospy.Subscriber("global_occupancy_grid", OccupancyGrid, update_occupancy_grid)

def send_robot_state(req):
    global odometry, occupancy_grid
    (s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11) = tuple(sensors)
    sensor_msg = sensor_value(portDriveEncoder=s0, starboardDriveEncoder=s1, bcAttitudePortPot=s2, bcAttitudeStarboardPot=s3,
                              depLoadCell=s4, excLoadCell=s5, depLowerLimit=s6, depUpperLimit=s7, excForeLimit=s8, excAftLimit=s9,
                              bcLowerLimit=s10, bcUpperLimit=s11)
    return RobotStateResponse(odometry=odometry, grid=occupancy_grid, sensors=sensor_msg, stamp=rospy.Time.now())

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
