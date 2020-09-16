#! /usr/bin/env python3
import rospy

import hwctrl.srv

if __name__ == '__main__':

    set_motor = rospy.ServiceProxy("set_motor", hwctrl.srv.SetMotor)
    while not rospy.is_shutdown():
        mtr_id= int(input("Motor ID: "))
        setpt = float(input("RPM setpoint: "))
        accel = float(input("Acceleration: "))
        req = hwctrl.srv.SetMotorRequest()
        req.id = mtr_id
        req.setpoint = setpt
        req.acceleration = accel
        resp = set_motor(req)
