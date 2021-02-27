#!/usr/bin/env python3
import rospy
from random import randint
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point 
from std_srvs.srv import Trigger

#Randomizes the robot state
def randomize_robot(req):
    robot_state = ModelState()
    robot_state.model_name = "robot"
    robot_state.pose.positions = Point(1, 3, 0.2)

#Randomizes the rock state 
def randomize_rocks(req):
    rock_1_state = ModelState()
    rock_1_state.model_name = "rock_1"
    rock_2_state = ModelState()
    rock_2_state.model_name = "rock_2"

    robot_state.pose.positions = Point(3, 3, 0.2)
    robot_state.pose.positions = Point(4, 3, 0.2)

    send_state(rock_1_state)
    send_state(rock_2_state)

#Sends the model state to gazebo
def send_state(model_state):
    rospy.loginfo("Waiting for set model state service...")
    rospy.wait_for_service("/gazebo/set_model_state")

    try:
        service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        res = service(model_state)
        rospy.loginfo("Successfully sent model state for %s"%(model_state.name))

    except rospy.ServiceException as e:
        rospy.logerr("Failed to send model state for %s"%(model_state.name))
        rospy.logerr(e)

#Listens for triggers to randomize the arena
def server():
    rospy.Service("/randomize_robot_state", Trigger, randomize_robot)
    rospy.Service("/randomize_rock_state", Trigger, randomize_rocks)

