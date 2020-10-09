#!/usr/bin/env python3

import rospy
import actionlib
from actionlib.msg import TestAction, TestGoal


class Excavator:
    def __init__(self):
        rospy.init_node("excavator_node")
        rospy.loginfo("Excavator node initialized")

        self.dump_client = actionlib.SimpleActionClient("dump", TestAction)

        rospy.loginfo("Waiting for dumping action server")
        self.dump_client.wait_for_server()
        rospy.loginfo("Dumping action server connected")

        self.server = actionlib.SimpleActionServer("dig", TestAction, self.dig, auto_start=False)
        self.server.start()

        rospy.spin()

    def dig(self, goal_msg):
        rospy.loginfo("Excavation received target goal: " + str(goal_msg.goal))

        # Send command to dig to fist depth

        # Send command to dump
        goal = TestGoal(goal=1)
        self.dump_client.send_goal(goal)
        self.dump_client.wait_for_result()

        # Send command to dig to second depth

        self.server.set_succeeded()
