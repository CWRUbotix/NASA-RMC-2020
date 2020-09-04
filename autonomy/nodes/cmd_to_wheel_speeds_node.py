#!/usr/bin/env python3
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from hwctrl.msg import SetMotorMsg

# Make these members of class later
failed = False
try:
    wheel_radius = rospy.get_param('wheel_radius')
    effective_robot_width = rospy.get_param("effective_robot_width")
except KeyError:
    failed = True


class CmdToWheelSpeedsNode:
    def __init__(self):
        rospy.init_node("cmd_to_wheel_speeds_node", anonymous=False)

        if failed:
            rospy.logerr("Parameters could not be found, please ensure they are on the param server")
            rospy.spin()

        self.motor_acceleration = rospy.get_param('/motor_command_accel')

        self.motor_setpoint_pub = rospy.Publisher("/motor_setpoints", SetMotorMsg, queue_size=2)
        rospy.Subscriber("/glennobi_diff_drive_controller/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

    def cmd_vel_callback(self, msg):
        vel = msg.linear.x
        angular_vel = msg.angular.z

        right_speed = (vel + angular_vel * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)
        left_speed = (vel - angular_vel * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)

        self.motor_setpoint_pub.publish(id=0, setpoint=left_speed, acceleration=self.motor_acceleration)
        self.motor_setpoint_pub.publish(id=1, setpoint=right_speed, acceleration=self.motor_acceleration)

if __name__ == "__main__":
    try:
        cmd_to_wheel_speeds_node = CmdToWheelSpeedsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
