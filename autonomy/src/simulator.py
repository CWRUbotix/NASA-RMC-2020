#!/usr/bin/env python3

import rospy
from hci.msg import sensorValue
from hci.msg import motorCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, Twist, TwistWithCovariance
from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
import numpy as np
from scipy.spatial.transform import Rotation as R


effective_robot_width = 0.7
wheel_radius = 0.2286


class Simulator:
    def __init__(self):
        self.robot = self.robot = SkidSteerSimulator(0.5, 0.5, 0)
        self.target_left_speed = 0
        self.target_right_speed = 0

        self.odometryPublisher = rospy.Publisher("odometry/filtered_map", Odometry, queue_size=1)
        self.sensorsPublisher = rospy.Publisher("hci/sensorValue", sensorValue, queue_size=4)

        print("Initializing simulator node")
        rospy.init_node("simulator", anonymous=False)
        print("Done initializing")
        self.subscribe()
        self.run()

    def run(self):
        rate = 20
        dt = 1.0/rate
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            target_vel = (self.target_right_speed + self.target_left_speed) * np.pi * wheel_radius / 60
            target_angular_vel = (self.target_right_speed - self.target_left_speed) * np.pi * wheel_radius / effective_robot_width / 30

            if self.robot.state_dot[0, 0] < target_vel:
                forward_torque = 30
            else:
                forward_torque = 0

            turn_torque = 7 * (target_angular_vel - self.robot.state_dot[2, 0])


            right_torque = forward_torque + turn_torque
            left_torque = forward_torque - turn_torque

            self.robot.update(right_torque, left_torque, dt)

            print("x:{:.2f}, y:{:.2f}, ang:{:.1f}, tr:{:.1f}, tl:{:.1f}".format(self.robot.state[0, 0],
                                                                               self.robot.state[1, 0],
                                                                               self.robot.state[2, 0],
                                                                                self.target_right_speed, self.target_left_speed))

            self.publishOdometry()
            self.publishSensorValues()

            r.sleep()

    def subscribe(self):
        rospy.Subscriber("motorCommand", motorCommand, self.receiveMotorCommand)

    def receiveMotorCommand(self, msg):
        if msg.motorID == 0:
            self.target_left_speed = msg.value
        elif msg.motorID == 1:
            self.target_right_speed = msg.value

    def publishOdometry(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        msg = Odometry()
        msg.header = header
        msg.child_frame_id = "base_link"

        pose = Pose()
        pose.position = Point(self.robot.state[0, 0], self.robot.state[1, 0], 0)
        quat = R.from_euler('z', self.robot.state[2, 0]).as_quat()
        pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        msg.pose = PoseWithCovariance(pose=pose)

        twist = Twist()
        twist.linear.x = self.robot.state_dot[0, 0]
        twist.angular.z = self.robot.state_dot[2, 0]
        msg.twist = TwistWithCovariance(twist=twist)

        self.odometryPublisher.publish(msg)

    def publishSensorValues(self):
        right_speed = (self.robot.state_dot[0, 0] + self.robot.state_dot[2, 0] * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)
        left_speed = (self.robot.state_dot[0, 0] - self.robot.state_dot[2, 0] * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)

        sensor_data = sensorValue(sensorID=0, value=left_speed)
        self.sensorsPublisher.publish(sensor_data)
        sensor_data = sensorValue(sensorID=1, value=right_speed)
        self.sensorsPublisher.publish(sensor_data)


if __name__ == "__main__":
    try:
        simulator = Simulator()
    except rospy.ROSInterruptException:
        pass
