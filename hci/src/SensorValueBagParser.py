#!/usr/bin/env python
import rospy
from hci.msg import sensorValue
import pandas as pd


sensorValueTopic = 'sensorValue'
sensorTable = {'deltaTime': [], 'sensorID': [], 'value': []}

def sensorValueCallback(data):

	if data.sensorID == 100:
		df = pd.DataFrame(sensorTable)
		df.to_csv('sensors7.csv', index=False)
		return


	currentTime = rospy.Time.now()
	deltaTime = currentTime - startTime
	rospy.loginfo("Sensor %u has value %f at time %f", data.sensorID, data.value, deltaTime.to_sec())
	sensorTable['deltaTime'].append(deltaTime.to_sec())
	sensorTable['sensorID'].append(data.sensorID)
	sensorTable['value'].append(data.value)


	


if __name__=='__main__':
	global startTime
	rospy.init_node('sensorValueParser')
	rospy.Subscriber(sensorValueTopic,sensorValue,sensorValueCallback)
	startTime = rospy.Time.now()
	rospy.spin()
	
	
	
