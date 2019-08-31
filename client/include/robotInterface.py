#!/usr/bin/env python 
import rospy

from hci.msg import sensorValue
from hci.srv import motorCommand


node_name = 'robotInterface'
motorCommandTopic = 'motorCommand'
sensorValueTopic = 'sensorValue'

motorCommandPub = None

sensorValueMap = {
	(0,0),
	(1,0),
	(2,0),
	(3,0),
	(4,0),
	(5,0),
	(6,0),
	(7,0),
	(8,0),
	(9,0),
	(10,0),
	(11,0),
	(12,0),
	(13,0),
	(14,0),
	(15,0),
	(16,0),
	(17,0),
	(18,0),
	(19,0),
	(20,0),
	(21,0),
	(22,0),
	(23,0),
	(24,0),
	(25,0),
	(26,0),
	(27,0),
	(28,0),
	(29,0),
	(30,0),
	(31,0),
	(32,0)
}


def sendMotorCommand(motorID, value):
	if motorCommandPub is None:
		return
	try:
		resp = motorCommandPub(motorID,value)
	except rospy.ServiceException as exc:
		print("motor command service didn't process request: " + str(exc))
	return resp.success

def sensorValueCallback(data):
	rospy.loginfo("Sensor %u has value %f", data.sensorID, data.value)
	sensorValueMap[data.sensorID] = data.value;

def getSensorValue(sensorID):
	return sensorValueMap(sensorID);

def initializeRobotInterface():
	#rospy.init_node(node_name,disable_signals=True)

	rospy.wait_for_service(motorCommandTopic)
	motorCommandPub = rospy.ServiceProxy(motorCommandTopic, motorCommand, persistent=True)

	rospy.Subscriber(sensorValueTopic,sensorValue,sensorValueCallback)
	rospy.spin()
