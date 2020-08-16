#!/usr/bin/env python3
import rospy
import actionlib
from scipy.spatial.transform import Rotation as R
from hwctrl.msg import SetMotorMsg
from autonomy.msg import GoToGoalAction, TransitPath, TransitControlData
from autonomy.srv import RobotState
from PathFollowing.PathFollower import PathFollower
from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
from PathPlanning.PathPlanningUtils import Position, Grid
import PathFollowing.config as config
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os
import sys
import glob
from enum import Enum

matplotlib.use('Agg')  # necessary when plotting without $DISPLAY


# Constants
ARENA_WIDTH = rospy.get_param('arena_x')
ARENA_HEIGHT = rospy.get_param('arena_y')
ROBOT_WIDTH = rospy.get_param('robot_width')
ROBOT_LENGTH = rospy.get_param('robot_length')
wheel_radius = rospy.get_param('wheel_radius')
config.G_u = rospy.get_param("autonomy/G_u")
config.lambda_e = rospy.get_param("autonomy/lambda_e")
config.target_velocity = rospy.get_param("autonomy/target_velocity")
config.turn_speed = rospy.get_param("autonomy/turn_speed")
reference_point_x = rospy.get_param("autonomy/reference_point_x")
effective_robot_width = rospy.get_param("effective_robot_width")


class State(Enum):
    FOLLOWING = 0
    BLOCKED_PAUSE = 1
    PREEMPTED = 2


class TransitNode:
    def __init__(self, visualize=False):
        self.server = actionlib.SimpleActionServer('go_to_goal', GoToGoalAction, self.go_to_goal, auto_start=False)

        self.motor_acceleration = rospy.get_param('/motor_command_accel')
        self.motor_pub = rospy.Publisher("motor_setpoints", SetMotorMsg, queue_size=4)
        self.path_pub = rospy.Publisher("transit_path", TransitPath, queue_size=4)
        self.control_data_pub = rospy.Publisher("transit_control_data", TransitControlData, queue_size=4)
        self.controller = PathFollower(reference_point_x, goal=(0, 0), config=config)
        self.robot_state = dict(state=np.array([[0, 0, 0]]).T, state_dot=np.array([[0, 0, 0]]).T)
        self.controller.update_grid(Grid(ARENA_WIDTH, ARENA_HEIGHT))

        self.state = State.FOLLOWING

        self.viz_dir = "transit_viz/"
        self.visualize = visualize
        self.viz_step = 15
        self.step = 0
        self.target_vels = []
        self.vels = []
        self.target_ang_vels = []
        self.ang_vels = []
        self.target_wheel_speeds = []
        self.wheel_speeds = []

        print("Creating directory: " + os.path.abspath(self.viz_dir))
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
        self.server.start()
        self.subscribe()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def go_to_goal(self, goal_msg):
        goal = Position(goal_msg.x, goal_msg.y)
        rospy.loginfo("Received ({}, {})".format(goal.x, goal.y))

        msg = self.get_robot_state()

        self.receive_state(msg.odometry)
        self.receive_grid(msg.grid)

        self.state = State.FOLLOWING
        self.controller.reset()
        self.controller.set_drive_backwards(True)
        self.controller.set_goal(goal)
        self.controller.state = self.robot_state["state"]
        self.controller.state_dot = self.robot_state["state_dot"]
        self.controller.calculate_path()
        self.publish_path()

        rospy.loginfo("Going to ({}, {})".format(goal.x, goal.y))

        rate = 30
        r = rospy.Rate(rate)  # 'rate' Hz
        last_time = rospy.get_time()
        last_pause_time = 0
        while not self.controller.done and not self.state == State.PREEMPTED:
            msg = self.get_robot_state()
            self.receive_state(msg.odometry)

            time = rospy.get_time()

            vel, angular_vel = 0, 0

            if self.server.is_preempt_requested():
                self.state = State.PREEMPTED

            if self.step % int(0.5 * rate) == 0 and self.state != State.PREEMPTED:  # Every half second
                self.receive_grid(msg.grid)
                if self.controller.is_path_blocked() and self.state == State.FOLLOWING:
                    self.state = State.BLOCKED_PAUSE
                    rospy.loginfo("Path blocked, regenerating")
                    last_pause_time = rospy.get_time()

                rospy.loginfo("Currently at: {:.2f}".format(self.controller.current_index))

            if self.state == State.FOLLOWING:
                vel, angular_vel = self.controller.get_target_vels(self.robot_state["state"],
                                                                   self.robot_state["state_dot"], time - last_time)
            elif self.state == State.BLOCKED_PAUSE:
                vel = 0
                angular_vel = 0
                if time - last_pause_time > 2:
                    self.controller.calculate_path()
                    self.publish_path()
                    self.state = State.FOLLOWING
            elif self.state == State.PREEMPTED:
                vel = 0
                angular_vel = 0
                rospy.loginfo("Preempt requested")
                self.server.set_preempted()

            right_speed = (vel + angular_vel * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)
            left_speed = (vel - angular_vel * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)

            self.target_vels.append(vel)
            self.target_ang_vels.append(angular_vel)
            self.vels.append(self.robot_state["state_dot"][0, 0])
            self.ang_vels.append(self.robot_state["state_dot"][2, 0])
            self.target_wheel_speeds.append([right_speed, left_speed])
            self.wheel_speeds.append([msg.sensors.starboardDriveEncoder, msg.sensors.portDriveEncoder])

            self.motor_pub.publish(id=0, setpoint=left_speed, acceleration=self.motor_acceleration)
            self.motor_pub.publish(id=1, setpoint=right_speed, acceleration=self.motor_acceleration)

            self.publish_control_data()

            self.draw()
            self.step += 1
            last_time = time
            r.sleep()

        if self.state != State.PREEMPTED:
            self.server.set_succeeded()
            rospy.loginfo("Path Complete")

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
        if msg.header.frame_id:  # Verify that the grid was received by checking non-empty
            grid = Grid(ARENA_WIDTH, ARENA_HEIGHT, occupancies=msg)  # Create a grid and add the obstacles to it
            self.controller.update_grid(grid)

    def subscribe(self):
        # rospy.Subscriber("robot_state", robotState, self.receive_state)
        # rospy.Subscriber("occupancy_grid", OccupancyGrid, self.receive_grid)
        pass

    def publish_path(self):
        path = self.controller.get_path()
        self.path_pub.publish(path=path.ravel())

    def publish_control_data(self):
        followed_segment, closest_point, reference_point = self.controller.draw_path_info()

        data = TransitControlData()
        data.t_vel = self.target_vels[-1]
        data.t_angular_vel = self.target_ang_vels[-1]
        data.vel = self.vels[-1]
        data.angular_vel = self.ang_vels[-1]
        data.t_right_speed = self.target_wheel_speeds[-1][0]
        data.t_left_speed = self.target_wheel_speeds[-1][1]
        data.right_speed = self.wheel_speeds[-1][0]
        data.left_speed = self.wheel_speeds[-1][1]
        data.followed_segment = followed_segment.ravel()
        data.closest_point = closest_point
        data.reference_point = reference_point

        self.control_data_pub.publish(data)

    def shutdown(self):
        self.motor_pub.publish(id=0, setpoint=0, acceleration=self.motor_acceleration)
        self.motor_pub.publish(id=1, setpoint=0, acceleration=self.motor_acceleration)

    def draw(self):
        if self.visualize and self.step % self.viz_step == 0:
            fig = plt.figure(figsize=(12, 4))

            ax = plt.subplot(131)
            ax.axis('equal')
            ax.set_xlim([-2, 8])
            ax.set_ylim([-3, 7])

            points = SkidSteerSimulator.draw(self.robot_state["state"], ROBOT_WIDTH, ROBOT_LENGTH)
            path_aprox, closest_point, reference = self.controller.draw_path_info()
            path = self.controller.get_path()

            ax.plot([0, ARENA_WIDTH, ARENA_WIDTH, 0, 0],
                     [0, 0, ARENA_HEIGHT, ARENA_HEIGHT, 0], color='black', linewidth=0.5)  # draw field
            ax.scatter(points[:, 0], points[:, 1])
            ax.plot(path[:, 0], path[:, 1], linewidth=0.75)
            ax.scatter(reference[0, 0], reference[1, 0])
            ax.scatter(closest_point[0], closest_point[1])
            # ax.plot(path_aprox[:, 0], path_aprox[:, 1])
            ax.set_title("Arena")

            ax = plt.subplot(132)
            ax.plot(self.target_vels, label='target vels')
            ax.plot(self.vels, label='vels')
            ax.plot(self.target_ang_vels, label='target ang vels')
            ax.plot(self.ang_vels, label='ang vels')
            ax.set_title('Vels')
            ax.legend()

            ax = plt.subplot(133)
            ax.plot(self.target_wheel_speeds, label='target wheels')
            ax.plot(self.wheel_speeds, label='wheels')
            ax.set_title('Wheels')
            ax.legend()

            fig.savefig(self.viz_dir + 'fig_' + str(int(self.step / self.viz_step)))
            plt.close(fig)


if __name__ == "__main__":
    try:
        visualize = (sys.argv[1] == "true")  # defaults to False
    except IndexError:
        visualize = False

    try:
        transit_node = TransitNode(visualize=visualize)
    except rospy.ROSInterruptException:
        pass
