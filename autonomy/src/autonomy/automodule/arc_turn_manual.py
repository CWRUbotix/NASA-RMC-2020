#!/usr/bin/env python3

import rospy
import math
from graphics import *
from hwctrl.msg import MotorCommand

robot_speed = 20
motor_pub = rospy.Publisher("motor_setpoints", MotorCommand, queue_size=100)
win = GraphWin("Turn Control", 500, 500)
speed_text = Text(Point(75, 75), str(robot_speed))
speed_text.draw(win)

def updateControl(event):
    x, y = event.widget.winfo_pointerxy()
    x = x - win.winfo_rootx() - win.winfo_width() / 2
    y = y - win.winfo_rooty() - win.winfo_height() / 2
    theta = -math.atan2(y, x)
    if math.fabs(theta-math.pi / 2) < math.pi / 12:
        left = 1
        right = 1
    elif theta >= math.pi / 2:
        right = 1
        left = math.cos(theta-math.pi / 2)
    elif theta >= 0:
        right = math.fabs(math.cos(theta - math.pi / 2))
        left = 1
    elif theta < -math.pi / 2:
        right = math.cos(theta-math.pi / 2)
        left = -1
    elif math.fabs(theta + math.pi / 2) < math.pi / 2:
        left = -1
        right = -1
    else:
        right = -1
        left = -math.cos(theta-math.pi / 2)
   # print(theta)
    print("left:", left * robot_speed)
    print("right:", right * robot_speed)
    motor_pub.publish(motorID=0, value = left * robot_speed)
    motor_pub.publish(motorID=1, value = right * robot_speed)
    
def stop(event):
    global robot_speed
    robot_speed = 0
    speed_text.setText(str(robot_speed))
    motor_pub.publish(motorID=0, value=0)
    motor_pub.publish(motorID=1, value=0)
    print(robot_speed)

def launchControl():
    global robot_speed
    win.setBackground("white")
    c = Circle(Point(250,250), 50)
    c.draw(win)
    win.bind("<B1-Motion>", updateControl)
    win.bind("<ButtonRelease-1>", updateControl)
    win.bind("<ButtonPress-3>", stop)
    while not rospy.is_shutdown():
        key = win.getKey()
        if key == "w":
            robot_speed += 5
        elif key == "s":
            robot_speed -= 5
        speed_text.setText(str(robot_speed))

def shutdown():
    win.close()
    motor_pub.publish(motorID=0, value=0)
    motor_pub.publish(motorID=1, value=0)

def main():
    rospy.init_node("ArcTurnControl")
    rospy.on_shutdown(shutdown)
    launchControl()
    rospy.spin()

if __name__ == "__main__":
    main()


