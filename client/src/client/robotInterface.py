#!/usr/bin/env python
import rospy
from hwctrl.msg import SensorData
from hwctrl.msg import MotorCmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

node_name = 'robot_interface'
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

def sendDepositionBucketSpeed(value, accel = 35):
    motor_msg = MotorCmd()
    motor_msg.setpoint = value
    motor_msg.acceleration = accel
    sendMotorCommand("/dumper/motor_cmd", motor_msg, "deposition bucket speed")

def sendConveyorSpeed(value, accel = 35):
    motor_msg = MotorCmd()
    motor_msg.setpoint = value
    motor_msg.acceleration = accel
    sendMotorCommand("/excavation/conveyor_cmd", motor_msg, "conveyor speed")

def sendExcavationDepth(value, accel = 35):
    motor_msg = MotorCmd()
    motor_msg.setpoint = value
    motor_msg.acceleration = accel
    sendMotorCommand("/excavation/depth_cmd", motor_msg, "excavation depth")

def sendConveyorAngle(value, accel = 35):
    motor_msg = MotorCmd()
    motor_msg.setpoint = value
    motor_msg.acceleration = accel
    sendMotorCommand("/excavation/angle_cmd", motor_msg, "conveyor angle")

def sendWheelSpeed(forward_vel):
    motor_msg = Twist()
    motor_msg.linear = Vector3(forward_vel, 0, 0)

    try:
        pub = rospy.Publisher('/glenn_base/cmd_vel', Twist, queue_size=1)
        pub.publish(motor_msg)
    except rospy.ROSInterruptException as e:
        print(e.getMessage())
        pass
    return True

def sendDriveCommand(direction, forward_vel, angular_vel = 35):
    motor_msg = Twist()

    if direction == 0:  # forward
        motor_msg.linear = Vector3(forward_vel, 0, 0)
        motor_msg.angular = Vector3(0, 0, 0)
    elif direction == 1:  # backward
        motor_msg.linear = Vector3(-forward_vel, 0, 0)
        motor_msg.angular = Vector3(0, 0, 0)
    elif direction == 2:  # right
        motor_msg.linear = Vector3(0, 0, 0)
        motor_msg.angular = Vector3(0, 0, -angular_vel)
    elif direction == 3:  # left
        motor_msg.linear = Vector3(0, 0, 0)
        motor_msg.angular = Vector3(0, 0, angular_vel)
    try:
        pub = rospy.Publisher('/glenn_base/cmd_vel', Twist, queue_size=1)
        pub.publish(motor_msg)
    except rospy.ROSInterruptException as e:
        print(e.getMessage())
        pass
    return True

#Sends a MotorCmd message on the given topic 
def sendMotorCommand(topic, motor_msg, msg_for):
    try:
        pub = rospy.Publisher(topic, MotorCmd, queue_size = 1)
        pub.publish(motor_msg)
        print("Sent motor command for ", msg_for, "with value: ", motor_msg.setpoint)
    except rospy.ROSInterruptException as e:
        print("There was a problem sending the motor command ", e.getMessage())
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
