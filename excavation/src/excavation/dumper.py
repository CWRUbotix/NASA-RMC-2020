#!/usr/bin/env python3

from enum import Enum

import rospy
import actionlib
from actionlib.msg import TestAction


# State Machine states
class State(Enum):
    RAISE = 0
    WAIT = 1
    LOWER = 2
    LIFT_A_BIT = 3


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

        self.move_speed = 0.5  # Speed to move dumper at
        self.wait_time = 2  # How long to let dirt fall out for

        # Variables that are part of state machine operation
        self.state_last_time = rospy.get_time()
        self.state_done = False

        # Create dump action server
        self.server = actionlib.SimpleActionServer("dump", TestAction, self.dump, auto_start=False)
        self.server.start()

        rospy.spin()

    def dump(self, goal_msg):
        rospy.loginfo("Dumper received target goal: " + str(goal_msg.goal))

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
                    self.state = State.LOWER

            elif self.state == State.LOWER:
                self.set_motor_speed(-self.move_speed)

                if self.weight_sensor_data > 0.1:
                    self.state = State.LIFT_A_BIT

            elif self.state == State.LIFT_A_BIT:
                self.set_motor_speed(self.move_speed)

                if self.weight_sensor_data < 0.05:
                    self.state_done = True

            # Check if state has changed
            if self.state != last_state:
                rospy.loginfo("Dump state machine %s -> %s" % (last_state, self.state))
                self.state_last_time = rospy.get_time()

            rate.sleep()  # Wait for desired time

        self.server.set_succeeded()

    def set_motor_speed(self, command):
        """Sends command to dumper motor"""
        pass
