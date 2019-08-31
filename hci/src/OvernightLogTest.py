#!/usr/bin/env python
import rospy



if __name__=='__main__':
	global startTime
	rospy.init_node('overnightLogTest')
	while not rospy.is_shutdown():
		rospy.sleep(60)
		rospy.loginfo("Still alive")
	rospy.spin()
	
	
	
