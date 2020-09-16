#!/usr/bin/env python3
import rospy
import actionlib
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from glenn_msgs.msg import GoToGoalAction, TransitControlData
from autonomy.path_following.path_follower import PathFollower
from autonomy.path_following.skid_steer_simulator import SkidSteerSimulator
from autonomy.path_planning.path_planning_utils import Position, Grid
import autonomy.path_following.config as config
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os
import sys
import glob
from enum import Enum

matplotlib.use('Agg')  # necessary when plotting without $DISPLAY


# Constants
failed = False
try:
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
except KeyError:
    # I believe that the init node should go above this but then it wouldn't be in the class
    # but I don't want these to have to be class members so for now I'm using this failed variable
    failed = True


class State(Enum):
    FOLLOWING = 0
    BLOCKED_PAUSE = 1
    PREEMPTED = 2


class TransitNode:
    def __init__(self, visualize=False):
        rospy.init_node("transit_node", anonymous=False)

        if failed:
            rospy.logerr("Parameters could not be found, please ensure they are on the param server")
            rospy.spin()

        self.server = actionlib.SimpleActionServer('go_to_goal', GoToGoalAction, self.go_to_goal, auto_start=False)

        self.command_vel_pub = rospy.Publisher("/glennobi_diff_drive_controller/cmd_vel", Twist, queue_size=4)
        self.path_pub = rospy.Publisher("transit_path", Path, queue_size=4)
        self.control_data_pub = rospy.Publisher("transit_control_data", TransitControlData, queue_size=4)
        self.controller = PathFollower(reference_point_x, goal=(0, 0), config=config)
        self.robot_state = dict(state=np.array([[0, 0, 0]]).T, state_dot=np.array([[0, 0, 0]]).T)
        self.grid = Grid(ARENA_WIDTH, ARENA_HEIGHT)
        self.controller.update_grid(self.grid)

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
        self.encoder_values = [0, 0]

        print("Creating directory: " + os.path.abspath(self.viz_dir))
        os.makedirs(self.viz_dir, exist_ok=True)
        try:
            files = glob.glob('%s/*' % self.viz_dir)
            for f in files:
                os.remove(f)
        except Exception as e:
            print(e)

        self.server.start()
        self.subscribe()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def go_to_goal(self, goal_msg):
        goal = Position(goal_msg.x, goal_msg.y)
        rospy.loginfo("Received ({}, {})".format(goal.x, goal.y))

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
            time = rospy.get_time()

            vel, angular_vel = 0, 0

            if self.server.is_preempt_requested():
                self.state = State.PREEMPTED

            if self.step % int(0.5 * rate) == 0 and self.state != State.PREEMPTED:  # Every half second
                self.controller.update_grid(self.grid)
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

            # only used for target wheel speeds for now, not published directly
            right_speed = (vel + angular_vel * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)
            left_speed = (vel - angular_vel * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)

            self.target_vels.append(vel)
            self.target_ang_vels.append(angular_vel)
            self.vels.append(self.robot_state["state_dot"][0, 0])
            self.ang_vels.append(self.robot_state["state_dot"][2, 0])
            self.target_wheel_speeds.append([right_speed, left_speed])
            self.wheel_speeds.append(self.encoder_values)

            self.publish_command_vel(vel, angular_vel)
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
            self.grid = Grid(ARENA_WIDTH, ARENA_HEIGHT, occupancies=msg)  # Create a grid and add the obstacles to it

    def recieve_motor_data(self, msg):
        if msg.id == 0 and msg.data_type == 0:  # port message and RPM value
            self.encoder_values[1] = msg.value
        if msg.id == 1 and msg.data_type == 0:  # starboard message and RPM value
            self.encoder_values[0] = msg.value

    def subscribe(self):
        rospy.Subscriber("odometry/filtered_map", Odometry, self.receive_state)
        rospy.Subscriber("global_occupancy_grid", OccupancyGrid, self.receive_grid)

    def publish_path(self):
        path = self.controller.get_path()
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for i, point in enumerate(path):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.seq = i
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

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

    def publish_command_vel(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        self.command_vel_pub.publish(twist_msg)

    def shutdown(self):
        self.command_vel_pub.publish(Twist())  # publish all 0s

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
