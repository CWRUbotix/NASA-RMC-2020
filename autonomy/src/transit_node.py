#!/usr/bin/env python3
import rospy
# from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from hci.msg import motorCommand
from autonomy.msg import goToGoal, transitPath
from nav_msgs.msg import OccupancyGrid
from autonomy.srv import RobotState
from uwb_localization.srv import EffectiveRPM
from PathFollowing.PathFollower import PathFollower
from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
from PathPlanning.PathPlanningUtils import Position, Grid
import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt
import os
import glob
matplotlib.use('Agg')  # necessary when plotting without $DISPLAY


# Constants
ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38
ROBOT_WIDTH = 0.63
ROBOT_LENGTH = 1.3
effective_robot_width = 0.8
reference_point_x = 0.3
wheel_radius = 0.2286


class TransitNode:
    def __init__(self, visualize=False):
        self.motor_pub = rospy.Publisher("motorCommand", motorCommand, queue_size=100)
        self.path_pub = rospy.Publisher("transitPath", transitPath, queue_size=4)

        self.controller = PathFollower(reference_point_x, goal=(0, 0))
        self.robot_state = dict(state=np.array([[0, 0, 0]]).T, state_dot=np.array([[0, 0, 0]]).T)
        self.controller.update_grid(Grid(ARENA_WIDTH, ARENA_HEIGHT))

        self.viz_dir = "visualizations/"
        self.visualize = visualize
        self.viz_step = 15
        self.step = 0

        self.prev_left_rpm = 0
        self.prev_right_rpm = 0

        os.makedirs(self.viz_dir, exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

        rospy.init_node("transit_node", anonymous=False)
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('robot_state')
        self.get_robot_state = rospy.ServiceProxy('robot_state', RobotState)
        rospy.loginfo("Services acquired")
        self.subscribe()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def go_to_goal(self, msg):
        goal = Position(msg.x, msg.y)

        self.controller.reset()
        self.controller.set_goal(goal)
        self.controller.calculate_path()

        rate = 30
        r = rospy.Rate(rate)  # 'rate' Hz
        last_time = rospy.get_rostime().nsecs
        while not rospy.is_shutdown() and not self.controller.done:
            msg = self.get_robot_state()
            self.receive_state(msg.odometry)
            if self.step % rate == 0:
                #self.receive_grid(msg.grid)
                pass

            time = rospy.get_rostime().nsecs
            dt = (time - last_time) * 1e-9

            vel, angular_vel = self.controller.get_target_vels(self.robot_state["state"],
                                                               self.robot_state["state_dot"], dt)

            right_speed = (vel + angular_vel * effective_robot_width / 2) * 120 * np.pi * wheel_radius
            left_speed = (vel - angular_vel * effective_robot_width / 2) * 120 * np.pi * wheel_radius
            right_acce = (right_speed - self.prev_right_rpm) / ((time - last_time) * 1e-9)
            left_acce = (left_speed - self.prev_left_rpm) / ((time - last_time) * 1e-9)

            
            '''
            goal_right = right_speed
            goal_left = left_speed

            rospy.wait_for_service("effective_RPM")
            effective_rpm = rospy.ServiceProxy("effective_RPM", EffectiveRPM)
            true_rpm = effective_rpm(left=left_speed, right=right_speed)
            num_iter = 50

            while num_iter > 0 and (math.fabs(true_rpm.effectiveLeft - goal_left) > 0.1 or math.fabs(true_rpm.effectiveRight - goal_right) > 0.1):
                num_iter -= 1
                left_diff = goal_left - true_rpm.effectiveLeft
                right_diff = goal_right - true_rpm.effectiveRight
                left_speed += 0.025 * left_diff
                right_speed += 0.025 * right_diff
                true_rpm = effective_rpm(left=left_speed, right=right_speed)
            '''
            self.motor_pub.publish(motorID=0, value=left_speed)
            self.motor_pub.publish(motorID=1, value=right_speed)

            self.draw()
            self.step += 1
            last_time = time
            r.sleep()

    def receive_state(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        o = pose.orientation
        angle = R.from_quat([o.x, o.y, o.z, o.w]).as_euler('zyx')[0]
        state = np.array([[pose.position.x, pose.position.y, angle]]).reshape(3, 1)
        state_dot = np.array([[twist.linear.x, twist.linear.y, twist.angular.z]]).reshape(-1, 1)

        self.robot_state["state"] = state
        self.robot_state["state_dot"] = state_dot

    def receive_grid(self, msg):
        grid = Grid(ARENA_WIDTH, ARENA_HEIGHT, occupancies=msg)  # Create a grid and add the obstacles to it
        self.controller.update_grid(grid)
        self.controller.calculate_path()
        self.publish_path()

    def subscribe(self):
        rospy.Subscriber("transit_command", goToGoal, self.go_to_goal)
        # rospy.Subscriber("robot_state", robotState, self.receive_state)
        # rospy.Subscriber("occupancy_grid", OccupancyGrid, self.receive_grid)
        #pass


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

    def draw(self):
        if self.visualize and self.step % self.viz_step == 0:
            plt.clf()
            plt.axis('equal')
            axes = plt.gca()
            axes.set_xlim([-2, 8])
            axes.set_ylim([-3, 7])

            plt.plot([0, ARENA_WIDTH, ARENA_WIDTH, 0], [0, 0, ARENA_HEIGHT, ARENA_HEIGHT], color='black')  # draw field
            points = SkidSteerSimulator.draw(self.robot_state["state"], ROBOT_WIDTH, ROBOT_LENGTH)
            path_aprox, closest_point, reference = self.controller.draw_path_info()
            path = self.controller.get_path()

            plt.scatter(points[:, 0], points[:, 1])
            plt.plot(path[:, 0], path[:, 1])
            plt.scatter(reference[0, 0], reference[1, 0])
            plt.scatter(closest_point[0], closest_point[1])
            plt.plot(path_aprox[:, 0], path_aprox[:, 1])

            plt.savefig(self.viz_dir + 'fig_' + str(int(self.step / self.viz_step)))


if __name__ == "__main__":
    try:
        transit_node = TransitNode(visualize=True)
    except rospy.ROSInterruptException:
        pass
