#!/usr/bin/python3
import rospy
from autonomy.msg import goToGoal

goal_pub = rospy.Publisher("transit_command", goToGoal, queue_size=10)

def on_shut_down():
    global goal_pub
    print("sending stop command")
    goal_pub.publish(stop=True, x=0.0, y=0.0)

def main():
    rospy.init_node("autonomy")
    rospy.on_shutdown(on_shut_down)
    global goal_pub

    print("TEST 2019/11/24")
    print("Testing simple stuff for transit")

    x = float(input("Enter x-coordinate of where you want to go"))
    y = float(input("Enter y-coordinate of where you wnat to go"))

    input("Hit <Enter> when you are ready")

    goal_pub.publish(stop=False, x=x, y=y)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    main()
