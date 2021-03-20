#!/usr/bin/env python3
import rospy
from randomize_arena.randomize import server

if __name__ == "__main__":
    rospy.init_node("randomize_arena_node", anonymous=False)
    rospy.loginfo("Initialized randomize arena node...")
    server()
    rospy.spin()