#!/usr/bin/env python
import rospy
from hwctrl.msg import SensorData
from hwctrl.msg import SetMotorMsg

node_name = 'robot_interface'
motorCommandTopic = 'motor_setpoints'
sensorValueTopic = 'sensor_value'

motorCommandPub = None
driveCommandPub = None

sensorValueMap = {
    0:0,
    1:0,
    2:0,
    3:0,
    4:0,
    5:0,
    6:0,
    7:0,
    8:0,
    9:0,
    10:0,
    11:0,
    12:0,
    13:0,
    14:0,
    15:0,
    16:0,
    17:0,
    18:0,
    19:0,
    20:0,
    21:0,
    22:0,
    23:0,
    24:0,
    25:0,
    26:0,
    27:0,
    28:0,
    29:0,
    30:0,
    31:0,
    32:0
}


def sendMotorCommand(motorID, value, accel=35):
    motor_msg = SetMotorMsg()
    motor_msg.id = motorID
    motor_msg.setpoint = value
    motor_msg.acceleration = accel
    try:
        pub = rospy.Publisher(motorCommandTopic, SetMotorMsg, queue_size=1)
        pub.publish(motor_msg)
    except rospy.ROSInterruptException as e:
        print(e.getMessage())
        pass
    return True


def sendDriveCommand(direction, value, accel=35):
    left_msg = SetMotorMsg()
    right_msg = SetMotorMsg()
    left_msg.id = 0
    right_msg.id = 1
    left_msg.acceleration = accel
    right_msg.acceleration = accel
    if direction == 0:  # forward
        left_msg.setpoint = value
        right_msg.setpoint = value
    elif direction == 1:  # backward
        left_msg.setpoint = -value
        right_msg.setpoint = -value
    elif direction == 2:  # right
        left_msg.setpoint = value
        right_msg.setpoint = -value
    elif direction == 3:  # left
        left_msg.setpoint = -value
        right_msg.setpoint = value
    try:
        pub = rospy.Publisher(motorCommandTopic, SetMotorMsg, queue_size=2)
        pub.publish(left_msg)
        pub.publish(right_msg)
    except rospy.ROSInterruptException as e:
        print(e.getMessage())
        pass
    return True

def sensorValueCallback(data):
    rospy.loginfo("Sensor %u has value %f", data.sensor_id, data.value)
    sensorValueMap[data.sensor_id] = data.value;

def getSensorValue(sensor_id):
    return sensorValueMap(sensor_id);

def initializeRobotInterface():
    #rospy.init_node(node_name,disable_signals=True)
    rospy.Subscriber(sensorValueTopic,SensorData,sensorValueCallback)
    #rospy.spin()
