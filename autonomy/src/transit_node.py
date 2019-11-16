#!/usr/bin/env python3
import rospy
from hci.msg import motorCommand
from autonomy.msg import goToGoal, transitPath, robotState
from nav_msgs.msg import OccupancyGrid
from PathFollowing.PathFollower import PathFollower
from PathPlanning.PathPlanningUtils import Position, Grid
import numpy as np

# Constants
ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38
effective_robot_width = 0.75
reference_point_x = 0.3
wheel_radius = 0.1

motor_pub = rospy.Publisher("motorCommand", motorCommand, queue_size=100)
path_pub = rospy.Publisher("transitPath", transitPath, queue_size=4)
controller = PathFollower(reference_point_x, goal=(0, 0))

robot = dict(state=np.array([[0, 0, 0]]).T, state_dot=np.array([[0, 0, 0]]).T)


def go_to_goal(msg):
    if msg.dig:
        goal = Position(2, 6)
    else:
        goal = Position(0.5, 0.5)

    controller.reset()
    controller.set_goal(goal)
    controller.calculate_path()

    r = rospy.rate(50)  # 50 Hz
    last_time = rospy.get_rosTime().nsec
    while not rospy.is_shutdown() and not controller.done:
        time = rospy.get_rosTime().nsec
        dt = (time - last_time) * 1e-9

        vel, angular_vel = controller.get_target_vels(robot["state"], robot["state_dot"], dt)

        right_speed = (vel + angular_vel * effective_robot_width / 2) * 120 * np.pi * wheel_radius
        left_speed = (vel - angular_vel * effective_robot_width / 2) * 120 * np.pi * wheel_radius

        motor_pub.publish(motorID=0, value=left_speed)
        motor_pub.publish(motorID=1, value=right_speed)

        last_time = time
        r.sleep()


def recieve_state(msg):
    global robot
    state = np.array(msg.state).reshape(-1, 1)
    state_dot = np.array(msg.state_dot).reshape(-1, 1)

    robot["state"] = state
    robot["state_dot"] = state_dot


def recieve_grid(msg):
    grid = Grid(ARENA_WIDTH, ARENA_HEIGHT, occupancies=msg)  # Create a grid and add the obstacles to it
    controller.update_grid(grid)
    controller.calculate_path()
    publish_path()


def subscribe():
    rospy.Subscriber("transit_command", goToGoal, go_to_goal)
    rospy.Subscriber("robot_state", robotState, recieve_state)
    rospy.Subscriber("occupancy_grid", OccupancyGrid, recieve_grid)


def publish_path():
    path = controller.get_path()
    data = []
    for point in path:
        data.append(point[0, 0])
        data.append(point[1, 0])
    path_pub.publish(path=data)


def shutdown():
    motor_pub.publish(motorID=0, value=0)
    motor_pub.publish(motorID=0, value=0)


def main():
    rospy.init_node("transit_node")
    rospy.on_shutdown(shutdown)
    subscribe()
    rospy.spin()


if __name__ == "__main__":
    try:
        print("Node Started")
        main()
    except rospy.ROSInterruptException:
        pass
