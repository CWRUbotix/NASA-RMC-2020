#!/usr/bin/env python
import robotInterface
from hci.msg import motorCommand
#from client.msg import motorCommand

node_name = 'clientNode'


if __name__=='__main__':
	rospy.init_node(node_name)
	robotInterface.initializeRobotInterface()
	robotInterface.sendMotorCommand(0,100)
	robotInterface.getSensorValue(0);
