#!/usr/bin/env python
import robotInterface

node_name = 'client_node'


if __name__=='__main__':
	rospy.init_node(node_name)
	robotInterface.initializeRobotInterface()
	robotInterface.sendMotorCommand(0,100)
	robotInterface.getSensorValue(0);
