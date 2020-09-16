#!/usr/local/bin/python3.7

import rospy
from hwctrl.srv import SetMotor


if __name__ == "__main__":
	rospy.init_node("hwctrl_test")
	rospy.wait_for_service("SetMotor")
	print("Found SetMotor service")
	set_motor = rospy.ServiceProxy("SetMotor", SetMotor)

	while True:
		try:
			left_vel = float(input("input desired left rpm: "))
			left_acce = float(input("input desired left acce: "))
			right_vel = float(input("input desired right rpm: "))
			right_acce = float(input("input desired right acce: "))
		except:
			break

		set_motor(id=0, setpoint=left_vel, acceleration=left_acce)
		set_motor(id=1, setpoint=right_vel, acceleration=right_acce)

	set_motor(id=0, setpoint=0.0, acceleration=70)
	set_motor(id=1, setpoint=0.0, acceleration=70)

