#!/usr/bin/env python3
import rospy
from hci.msg import motorCommand
from autonomy.msg import goToGoal, transitPath, robotState
from nav_msgs.msg import OccupancyGrid
from PathFollowing.PathFollower import PathFollower
from PathPlanning.PathPlanningUtils import Position, Grid
import numpy as np
import os
import glob

# Constants
ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38
effective_robot_width = 0.75
reference_point_x = 0.3
wheel_radius = 0.1


class TransitNode:
    def __init__(self, visualize=False):
        self.motor_pub = rospy.Publisher("motorCommand", motorCommand, queue_size=100)
        self.path_pub = rospy.Publisher("transitPath", transitPath, queue_size=4)

        self.controller = PathFollower(reference_point_x, goal=(0, 0))
        self.robot_state = dict(state=np.array([[0, 0, 0]]).T, state_dot=np.array([[0, 0, 0]]).T)

        self.viz_dir = "visualizations/"
        self.visualize = visualize
        self.viz_step = 10

        rospy.init_node("transit_node", anonymous=False)
        rospy.loginfo("Booting up node...")
        self.subscribe()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def go_to_goal(self, msg):
        if msg.dig:
            goal = Position(2, 6)
        else:
            goal = Position(0.5, 0.5)

        self.controller.reset()
        self.controller.set_goal(goal)
        self.controller.calculate_path()

        r = rospy.rate(50)  # 50 Hz
        last_time = rospy.get_rosTime().nsec
        while not rospy.is_shutdown() and not self.controller.done:
            time = rospy.get_rosTime().nsec
            dt = (time - last_time) * 1e-9

            vel, angular_vel = self.controller.get_target_vels(self.robot_state["state"],
                                                               self.robot_state["state_dot"], dt)

            right_speed = (vel + angular_vel * effective_robot_width / 2) * 120 * np.pi * wheel_radius
            left_speed = (vel - angular_vel * effective_robot_width / 2) * 120 * np.pi * wheel_radius

            self.motor_pub.publish(motorID=0, value=left_speed)
            self.motor_pub.publish(motorID=1, value=right_speed)

            last_time = time
            r.sleep()

    def receive_state(self, msg):
        state = np.array(msg.state).reshape(-1, 1)
        state_dot = np.array(msg.state_dot).reshape(-1, 1)

        self.robot_state["state"] = state
        self.robot_state["state_dot"] = state_dot

    def receive_grid(self, msg):
        grid = Grid(ARENA_WIDTH, ARENA_HEIGHT, occupancies=msg)  # Create a grid and add the obstacles to it
        self.controller.update_grid(grid)
        self.controller.calculate_path()
        self.publish_path()

    def subscribe(self):
        rospy.Subscriber("transit_command", goToGoal, self.go_to_goal)
        rospy.Subscriber("robot_state", robotState, self.receive_state)
        rospy.Subscriber("occupancy_grid", OccupancyGrid, self.receive_grid)

    def publish_path(self):
        path = self.controller.get_path()
        data = []
        for point in path:
            data.append(point[0, 0])
            data.append(point[1, 0])
        self.path_pub.publish(path=data)

    def shutdown(self):
        self.motor_pub.publish(motorID=0, value=0)
        self.motor_pub.publish(motorID=0, value=0)


if __name__ == "__main__":
    try:
        transit_node = TransitNode(visualize=False)
    except rospy.ROSInterruptException:
        pass
