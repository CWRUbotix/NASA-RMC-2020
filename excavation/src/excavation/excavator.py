#!/usr/bin/env python3

from enum import Enum

import rospy
import actionlib

from actionlib.msg import TestAction, TestGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Float32

from hwctrl.msg import SetMotorMsg


# State Machine states
class State(Enum):
    RAISE = 0
    EXTEND_DIG = 1
    DUMP = 2
    RETRACT = 3


class Excavator:
    def __init__(self):
        rospy.init_node("excavator_node")
        rospy.loginfo("Excavator node initialized")

        # Current state
        self.state = State.RAISE

        # Sensor values
        self.dumper_weight = 0
        self.excavation_depth = 0
        self.excavation_angle = 0

        # Parameters
        self.angle_setpoint = 1.45  # rad
        self.conveyor_cmd_setpoint = 0.9
        self.first_depth = 0.30  # m
        self.second_depth = 0.40  # m
        self.conveyor_accel = 1
        self.full_weight_threshold = 1.2  # kg

        # Variables that are part of state machine operation
        self.state_last_time = rospy.get_time()
        self.state_done = False
        self.has_dug_top_layer = False
        self.has_sent_dump_goal = False

        self.angle_cmd_pub = rospy.Publisher("excavation/angle_cmd", SetMotorMsg, queue_size=2)
        self.depth_cmd_pub = rospy.Publisher("excavation/depth_cmd", SetMotorMsg, queue_size=2)
        self.conveyor_cmd_pub = rospy.Publisher("excavation/conveyor_cmd", SetMotorMsg, queue_size=2)

        rospy.Subscriber("excavation/angle", Float32, self.receive_angle, queue_size=4)
        rospy.Subscriber("excavation/depth", Float32, self.receive_depth, queue_size=4)
        rospy.Subscriber("dumper/weight", Float32, self.receive_dumper_weight, queue_size=4)

        self.dump_client = actionlib.SimpleActionClient("dump", TestAction)

        rospy.loginfo("Waiting for dumping action server")
        self.dump_client.wait_for_server()
        rospy.loginfo("Dumping action server connected")

        self.server = actionlib.SimpleActionServer("dig", TestAction, self.dig, auto_start=False)
        self.server.start()

        rospy.spin()

    def dig(self, goal_msg):
        rospy.loginfo("Excavation received target goal: " + str(goal_msg.goal))

        # Initialize variables
        self.state = State.RAISE
        self.state_done = False
        self.state_last_time = rospy.get_time()
        self.has_dug_top_layer = False
        self.has_sent_dump_goal = False

        rate = rospy.Rate(15)  # Run control loop at 15 Hz

        while not rospy.is_shutdown() and not self.state_done:
            last_state = self.state  # Store state to check if it changes

            if self.state == State.RAISE:
                self.publish_cmd(self.angle_cmd_pub, self.angle_setpoint)

                # Check if excavator is within ~5 deg of setpoint
                if abs(self.excavation_angle - self.angle_setpoint) < 0.1:
                    self.state = State.EXTEND_DIG
                
            elif self.state == State.EXTEND_DIG:
                depth = self.second_depth if self.has_dug_top_layer else self.first_depth

                # Lower excavator and start conveyor
                self.publish_cmd(self.depth_cmd_pub, depth)
                self.publish_cmd(self.conveyor_cmd_pub, self.conveyor_cmd_setpoint)

                # Wait for dirt to fill bucket
                # Could also check for bucket weight not increasing
                if self.dumper_weight > self.full_weight_threshold:
                    self.state = State.RETRACT if self.has_dug_top_layer else State.DUMP
                    self.has_dug_top_layer = True
                    self.publish_cmd(self.conveyor_cmd_pub, 0) # Stop

            elif self.state == State.DUMP:
                if not self.has_sent_dump_goal:
                    goal = TestGoal(goal=1)
                    self.dump_client.send_goal(goal)
                    self.has_sent_dump_goal = True

                if self.dump_client.get_state() == GoalStatus.SUCCEEDED:
                    self.state = State.EXTEND_DIG

            elif self.state == State.RETRACT:
                self.publish_cmd(self.depth_cmd_pub, 0)

                # Wait for retraction
                if self.excavation_depth < 0.05:
                    self.state_done = True

            # Check if state has changed
            if self.state != last_state:
                rospy.loginfo("Excavation state machine %s -> %s" % (last_state, self.state))
                self.state_last_time = rospy.get_time()

            try:
                rate.sleep()  # Wait for desired time
            except rospy.ROSInterruptException as e:
                rospy.loginfo(str(e))

        self.server.set_succeeded()
        rospy.loginfo("Excavation state machine finished")

    def publish_cmd(self, publisher, command, accel=None):
        motor_msg = SetMotorMsg()
        motor_msg.setpoint = command
        if accel:
            motor_msg.acceleration = accel
        publisher.publish(motor_msg)

    def receive_angle(self, msg):
        self.excavation_angle = msg.data

    def receive_depth(self, msg):
        self.excavation_depth = msg.data

    def receive_dumper_weight(self, msg):
        self.dumper_weight = msg.data
