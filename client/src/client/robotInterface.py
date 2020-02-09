#!/usr/bin/env python 
import rospy

#from client.srv import motorCommand
#from client.msg import sensorValue
from hci.msg import sensorValue
from hci.msg import motorCommand
from hci.msg import driveCommand
import hwctrl.srv

set_motor = rospy.ServiceProxy("set_motor", hwctrl.srv.SetMotor)

node_name = 'robotInterface'
motorCommandTopic = 'motorCommand'
driveCommandTopic = 'driveCommand'
sensorValueTopic = 'sensorValue'

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
	req = hwctrl.srv.SetMotorRequest()
	req.id = motorID
	req.setpoint = value
	req.acceleration = accel
	resp = set_motor(req)
	return True


def sendDriveCommand(direction, value, accel=35):
	left_req = hwctrl.srv.SetMotorRequest()
	right_req = hwctrl.srv.SetMotorRequest()
	left_req.id = 0
	right_req.id = 1
	left_req.acceleration = accel
	right_req.acceleration = accel
	if direction == 0:  # forward
		left_req.setpoint = value
		right_req.setpoint = value
	elif direction == 1:  # backward
		left_req.setpoint = -value
		right_req.setpoint = -value
	elif direction == 2:  # right
		left_req.setpoint = value
		right_req.setpoint = -value
	elif direction == 3:  # left
		left_req.setpoint = -value
		right_req.setpoint = value
	return True

def sensorValueCallback(data):
	rospy.loginfo("Sensor %u has value %f", data.sensorID, data.value)
	sensorValueMap[data.sensorID] = data.value;

def getSensorValue(sensorID):
	return sensorValueMap(sensorID);

def initializeRobotInterface():
	#rospy.init_node(node_name,disable_signals=True)

	motorCommandPub = rospy.Publisher(motorCommandTopic, motorCommand, queue_size=10)
	driveCommandPub = rospy.Publisher(driveCommandTopic, driveCommand, queue_size=10)
	rospy.Subscriber(sensorValueTopic,sensorValue,sensorValueCallback)
	#rospy.spin()
