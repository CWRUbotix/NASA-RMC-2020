#!/usr/bin/env python3
import rospy
from random import uniform
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point 
from std_srvs.srv import Trigger, TriggerResponse

#Randomizes the robot state
def randomize_robot(req):
    robot_state = ModelState()
    robot_state.model_name = "robot"
    random_x = uniform(0, 2.5)
    random_y = uniform(0, 6.8)

    robot_state.pose.position = Point(random_x, random_y, 0.2)
    send_state(robot_state)

#Randomizes the rock state 
def randomize_rocks(req):
    rock_1_state = ModelState()
    rock_2_state = ModelState()
    rock_1_state.model_name = "rock_1"
    rock_2_state.model_name = "rock_2"

    #Randomize rock 1 
    rock1_x = uniform(0, 2.5)
    rock1_y = uniform(0, 3.6)

    #Randoize rock 2
    rock2_x = uniform(0, 2.5)
    rock2_y = uniform(0, 3.6)

    #Send to gazebo
    rock_1_state.pose.position = Point(rock1_x, rock1_y, 0.2)
    rock_2_state.pose.position = Point(rock2_x, rock2_y, 0.2)
    send_state(rock_1_state)
    send_state(rock_2_state)

#Sends the model state to gazebo
def send_state(model_state):
    rospy.loginfo("Waiting for set model state service...")
    rospy.wait_for_service("/gazebo/set_model_state")

    try:
        service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        res = service(model_state)
        rospy.loginfo("Successfully sent model state for %s"%(model_state.model_name))

    except rospy.ServiceException as e:
        rospy.logerr("Failed to send model state for %s"%(model_state.model_name))
        rospy.logerr(e)

#Listens for triggers to randomize the arena
def server():
    rospy.Service("/randomize_robot_state", Trigger, randomize_robot)
    rospy.Service("/randomize_rock_state", Trigger, randomize_rocks)

