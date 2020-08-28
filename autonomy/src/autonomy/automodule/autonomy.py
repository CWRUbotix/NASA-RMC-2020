#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from glenn_msgs.msg import GoToGoalAction, GoToGoalGoal


def main():
    rospy.init_node("autonomy")

    client = actionlib.SimpleActionClient('go_to_goal', GoToGoalAction)
    rospy.loginfo("Waiting for robot action server")
    client.wait_for_server()
    rospy.loginfo("Server found")

    while True:
        failure = True
        while failure:
            try:
                x = float(input("Enter x-coordinate of where you want to go\n"))
                y = float(input("Enter y-coordinate of where you want to go\n"))
                input("Hit <Enter> when you are ready")
                failure = False
            except ValueError:
                print("Oops enter a valid value\n")
                failure = True

        rospy.loginfo("Sent")

        goal = GoToGoalGoal(x=x, y=y)
        client.send_goal(goal)

        input("Press <Enter> to stop robot or continue")

        if client.get_state() == GoalStatus.ACTIVE:
            rospy.loginfo("Canceling action")
            client.cancel_goal()
        try:
            print("result: ", GoalStatus.to_string(client.get_state()))
        except Exception as e:
            print(e)

    while not rospy.is_shutdown():
        rospy.spin()
