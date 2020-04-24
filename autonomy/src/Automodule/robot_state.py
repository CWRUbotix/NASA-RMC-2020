#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from hwctrl.msg import MotorData, SensorValue
from autonomy.msg import ExtraSensorValues
from autonomy.srv import RobotState, RobotStateResponse

odometry = None
occupancy_grid = None
service = None
sensors = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

sensormap = {
    0  : 0,  # port encoder
    1  : 1,  # starboard encoder
    5  : 2,  # bc_attitude_port_pot
    6  : 3,  # bc_attitude_starboard_pot
    9  : 4,  # dep_load_cell
    10 : 5,  # exc_load_cell
    23 : 6,  # dep_lower_limit
    24 : 7,  # dep_upper_limit
    25 : 8,  # exc_fore_limit
    26 : 9, # exc_aft_limit
    27 : 10, # bc_lower_limit
    28 : 11  # bc_upper_limit
}

def update_odometry(msg):
    global odometry
    odometry = msg

def update_occupancy_grid(msg):
    global occupancy_grid
    occupancy_grid = msg

def update_sensors(msg):
    global sensors, sensormap
    if msg.sensor_id in sensormap.keys() and msg.id != 0 and msg.id != 1:
        sensors[sensormap[msg.sensor_id]] = msg.value

def update_encoders(msg):
    global sensors, sensormap
    if msg.id in sensormap.keys():
        if msg.id == 0 and msg.data_type == 0:  # port message and RPM value
            sensors[sensormap[msg.id]] = msg.value
        if msg.id == 1 and msg.data_type == 0:  # starboard message and RPM value
            sensors[sensormap[msg.id]] = msg.value



def subscribe():
    rospy.Subscriber("sensor_value", SensorValue, update_sensors)
    rospy.Subscriber("motor_data", MotorData, update_encoders)
    rospy.Subscriber("odometry/filtered_map", Odometry, update_odometry)
    rospy.Subscriber("global_occupancy_grid", OccupancyGrid, update_occupancy_grid)

def send_robot_state(req):
    global odometry, occupancy_grid
    (s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11) = tuple(sensors)
    sensor_msg = ExtraSensorValues(port_drive_encoder=s0, starboard_drive_encoder=s1, bc_attitude_port_pot=s2, bc_attitude_starboard_pot=s3,
                              dep_load_cell=s4, exc_load_cell=s5, dep_lower_limit=s6, dep_upper_limit=s7, exc_fore_limit=s8, exc_aft_limit=s9,
                              bc_lower_limit=s10, bc_upper_limit=s11)
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
