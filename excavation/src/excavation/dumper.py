#!/usr/bin/env python3

from enum import Enum

import rospy
import actionlib
from actionlib.msg import TestAction

from std_msgs.msg import Float32, Bool
from hwctrl.msg import SetMotorMsg


# State Machine states
class State(Enum):
    RAISE = 0
    WAIT = 1
    LOWER = 2
    MINI_LOWER = 3
    LIFT_A_BIT = 4


class Dumper:
    def __init__(self):
        rospy.init_node("dumper_node")
        rospy.loginfo("Dumper node initialized")

        # Current state
        self.state = State.RAISE

        # Variables for sensor data
        self.top_sensor_data = False
        self.weight_sensor_data = 0
        self.bucket_pos_data = 0

        self.move_speed = rospy.get_param("~move_speed")  # Speed to move dumper at
        self.wait_time = rospy.get_param("~wait_time")  # How long to let dirt fall out for
        self.max_accel = rospy.get_param("~max_accel")  # Max bucket acceleration
        self.mini_lower_time = rospy.get_param("~mini_lower_time")  # Mini lower duration

        # Variables that are part of state machine operation
        self.state_last_time = rospy.get_time()
        self.state_done = False

        self.motor_setpoint_pub = rospy.Publisher("dumper/motor_cmd", SetMotorMsg, queue_size=2)

        rospy.Subscriber("dumper/top_limit_switch", Bool, self.receive_top_limit_switch, queue_size=4)
        rospy.Subscriber("dumper/weight", Float32, self.receive_dumper_weight, queue_size=4)
        rospy.Subscriber("dumper/position", Float32, self.receive_dumper_pos, queue_size=4)

        # Create dump action server
        self.server = actionlib.SimpleActionServer("dump", TestAction, self.dump, auto_start=False)
        self.server.start()

        rospy.spin()

    def dump(self, goal_msg):
        rospy.loginfo("Dumper received target goal: " + str(goal_msg.goal))
        mini_lower_count = goal_msg.goal

        # Initialize variables
        self.state = State.RAISE
        self.state_done = False
        self.state_last_time = rospy.get_time()

        rate = rospy.Rate(15)  # Run control loop at 15 Hz

        while not rospy.is_shutdown() and not self.state_done:
            last_state = self.state  # Store state to check if it changes

            if self.state == State.RAISE:
                self.set_motor_speed(self.move_speed)

                if self.top_sensor_data:
                    self.state = State.WAIT

            elif self.state == State.WAIT:
                self.set_motor_speed(0)

                # Wait for determined time
                if rospy.get_time() - self.state_last_time > self.wait_time:
                    if mini_lower_count > 0:
                        self.state = State.MINI_LOWER
                    else:
                        self.state = State.LOWER

            elif self.state == State.LOWER:
                self.set_motor_speed(-self.move_speed)

                if self.weight_sensor_data > 0.1:
                    self.state = State.LIFT_A_BIT

            elif self.state == State.MINI_LOWER:
                self.set_motor_speed(-self.move_speed)

                if rospy.get_time() - self.state_last_time > self.mini_lower_time:
                    mini_lower_count -= 1  # We have completed one mini lower
                    self.state = State.RAISE

            elif self.state == State.LIFT_A_BIT:
                self.set_motor_speed(self.move_speed)

                if self.weight_sensor_data < 0.05:
                    self.state_done = True

            # Check if state has changed
            if self.state != last_state:
                rospy.loginfo("Dump state machine %s -> %s" % (last_state, self.state))
                self.state_last_time = rospy.get_time()

            rate.sleep()  # Wait for desired time

        self.set_motor_speed(0)
        rospy.loginfo("Dump state machine finished")
        self.server.set_succeeded()

    def set_motor_speed(self, command):
        motor_msg = SetMotorMsg()
        motor_msg.setpoint = command
        motor_msg.acceleration = self.max_accel
        self.motor_setpoint_pub.publish(motor_msg)

    def receive_top_limit_switch(self, msg):
        self.top_sensor_data = msg.data

    def receive_dumper_weight(self, msg):
        self.weight_sensor_data = msg.data

    def receive_dumper_pos(self, msg):
        self.bucket_pos_data = msg.data
