#!/usr/bin/env python
import rospy
import actionlib
from actionlib.msg import TestAction, TestGoal
from hwctrl.msg import SensorData
from hwctrl.msg import MotorCmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

node_name = 'robot_interface'
sensorValueTopic = 'sensor_value'

motorCommandPub = None
driveCommandPub = None
angular_vel_factor = 1

dep_bucket_speed_pub = rospy.Publisher('/dumper/motor_cmd', MotorCmd, queue_size=1)
excavation_speed_pub = rospy.Publisher('/excavation/conveyor_cmd', MotorCmd, queue_size=1)
excavation_depth_pub = rospy.Publisher('/excavation/depth_cmd', MotorCmd, queue_size=1)
excavation_angle_pub = rospy.Publisher('/excavation/angle_cmd', MotorCmd, queue_size=1)
drive_command_pub = rospy.Publisher('/glenn_base/cmd_vel', Twist, queue_size=1)
dig_action_client = actionlib.SimpleActionClient('dig', TestAction)
dump_action_client = actionlib.SimpleActionClient('dump', TestAction)

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

def sendDepositionBucketSpeed(value, accel=35):
    sendMotorCommand(dep_bucket_speed_pub, value, accel, "deposition bucket speed")

def sendExcavationSpeed(value, accel=35):
    sendMotorCommand(excavation_speed_pub, value, accel, "excavation speed")

def sendExcavationDepth(value, accel=35):
    sendMotorCommand(excavation_depth_pub, value, accel, "excavation depth")

def sendExcavationAngle(value, accel=35):
    sendMotorCommand(excavation_angle_pub, value, accel, "excavation angle")

def sendWheelSpeed(forward_vel):
    motor_msg = Twist()
    motor_msg.linear = Vector3(forward_vel, 0, 0)

    try:
        drive_command_pub.publish(motor_msg)
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)

def sendDriveCommand(direction, forward_vel):
    motor_msg = Twist()

    if direction == 0:  # forward
        motor_msg.linear = Vector3(forward_vel, 0, 0)
        motor_msg.angular = Vector3(0, 0, 0)
    elif direction == 1:  # backward
        motor_msg.linear = Vector3(-forward_vel, 0, 0)
        motor_msg.angular = Vector3(0, 0, 0)
    elif direction == 2:  # right
        motor_msg.linear = Vector3(0, 0, 0)
        motor_msg.angular = Vector3(0, 0, -forward_vel * angular_vel_factor)
    elif direction == 3:  # left
        motor_msg.linear = Vector3(0, 0, 0)
        motor_msg.angular = Vector3(0, 0, forward_vel * angular_vel_factor)
    try:
        drive_command_pub.publish(motor_msg)
        rospy.loginfo("Sent drive command with \nlinear: \n%s \nangular: \n%s", motor_msg.linear, motor_msg.angular)
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)

#Publishes a MotorCmd message on the given publisher 
def sendMotorCommand(pub, value, accel, msg_for):
    motor_msg = MotorCmd()
    motor_msg.setpoint = value
    motor_msg.acceleration = accel

    try:
        pub.publish(motor_msg)
        rospy.loginfo("Sent motor command for %s with value: %f", msg_for, motor_msg.setpoint)
    except rospy.ROSInterruptException as e:
        rospy.logwarn("There was a problem sending the motor command %s", e)

def sensorValueCallback(data):
    rospy.loginfo("Sensor %u has value %f", data.sensor_id, data.value)
    sensorValueMap[data.sensor_id] = data.value

def getSensorValue(sensor_id):
    return sensorValueMap.get(sensor_id)

def sendDumpAction(val):
    goal_msg = TestGoal(goal=val)
    connected = dump_action_client.wait_for_server(rospy.Duration(0.5))

    if connected:
        dump_action_client.send_goal(goal_msg)
    else:
        rospy.logwarn('Dumping action server failed to connect after 0.5 seconds')

def sendDigAction(val):
    goal_msg = TestGoal(goal=val)
    connected = dig_action_client.wait_for_server(rospy.Duration(0.5))
    
    if connected:
        dig_action_client.send_goal(goal_msg)
    else:
        rospy.logwarn('Excavation action server failed to connect after 0.5 seconds')


def initializeRobotInterface():
    #rospy.init_node(node_name,disable_signals=True)
    rospy.Subscriber(sensorValueTopic,SensorData,sensorValueCallback)
    #rospy.spin()
