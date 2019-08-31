#!/usr/bin/env python 
import rospy

#from client.srv import motorCommand
#from client.msg import sensorValue
from hci.msg import sensorValue
from hci.msg import motorCommand
from hci.msg import driveCommand


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


def sendMotorCommand(motorID, value):
	global motorCommandPub
	if motorCommandPub == None:
		motorCommandPub = rospy.Publisher(motorCommandTopic, motorCommand, queue_size=10)
		print("why are you like this")
	motorCommandPub.publish(motorID, value)
	return True

def sendDriveCommand(direction, value):
	global driveCommandPub
	if driveCommandPub == None:
		driveCommandPub = rospy.Publisher(driveCommandTopic, driveCommand, queue_size=10)
		print("why are you like this2")
	driveCommandPub.publish(direction, value)
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
