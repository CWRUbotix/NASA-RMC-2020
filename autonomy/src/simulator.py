#!/usr/bin/env python3

import rospy
from hwctrl.msg import MotorData, SetMotorMsg
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, Twist, TwistWithCovariance
from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
import numpy as np
from scipy.spatial.transform import Rotation as R


effective_robot_width = 0.7
wheel_radius = 0.2286
grid_resolution = 0.15
grid_size = 4


class Simulator:
    def __init__(self):
        self.robot = SkidSteerSimulator(1.5, 0.5, -np.pi/2)
        self.target_left_speed = 0
        self.target_right_speed = 0

        self.obstacles = np.array([[1, 3], [2.5, 2.5]], dtype='float64')

        self.camera_offset = [-0.4, 0, np.pi/2]

        self.odometryPublisher = rospy.Publisher("glennobi_diff_drive_controller/odom", Odometry, queue_size=1)
        self.sensorsPublisher = rospy.Publisher("motor_data", MotorData, queue_size=4)
        self.occupancyGridPublisher = rospy.Publisher("local_occupancy_grid", OccupancyGrid, queue_size=1)

        print("Initializing simulator node")
        rospy.init_node("simulator", anonymous=False)
        print("Done initializing")
        self.subscribe()
        self.run()

    def run(self):
        rate = 20
        dt = 1.0/rate
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            target_vel = (self.target_right_speed + self.target_left_speed) * np.pi * wheel_radius / 60
            target_angular_vel = (self.target_right_speed - self.target_left_speed) * np.pi * wheel_radius / effective_robot_width / 30

            if abs(self.robot.state_dot[0, 0]) < abs(target_vel):
                forward_torque = 30 * np.sign(target_vel)
            else:
                forward_torque = 0

            turn_torque = 7 * (target_angular_vel - self.robot.state_dot[2, 0])

            right_torque = forward_torque + turn_torque
            left_torque = forward_torque - turn_torque

            self.robot.update(right_torque, left_torque, dt)

            self.publishOdometry()
            self.publishSensorValues()
            self.publishOccupancyGrid()

            r.sleep()

    def subscribe(self):
        rospy.Subscriber("motor_setpoints", SetMotorMsg, self.receiveMotorCommand)

    def receiveMotorCommand(self, msg):
        if msg.id == 0:
            self.target_left_speed = msg.setpoint
        elif msg.id == 1:
            self.target_right_speed = msg.setpoint

    def publishOdometry(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        msg = Odometry()
        msg.header = header
        msg.child_frame_id = "base_link"

        pose = Pose()
        pose.position = Point(self.robot.state[0, 0], self.robot.state[1, 0], 0)
        quat = R.from_euler('z', self.robot.state[2, 0]).as_quat()
        pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        msg.pose = PoseWithCovariance(pose=pose)

        twist = Twist()
        twist.linear.x = self.robot.state_dot[0, 0]
        twist.angular.z = self.robot.state_dot[2, 0]
        msg.twist = TwistWithCovariance(twist=twist)

        self.odometryPublisher.publish(msg)

    def publishSensorValues(self):
        right_speed = (self.robot.state_dot[0, 0] + self.robot.state_dot[2, 0] * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)
        left_speed = (self.robot.state_dot[0, 0] - self.robot.state_dot[2, 0] * effective_robot_width / 2) * 30 / (np.pi * wheel_radius)

        sensor_data = MotorData(id=0, value=left_speed)
        self.sensorsPublisher.publish(sensor_data)
        sensor_data = MotorData(id=1, value=right_speed)
        self.sensorsPublisher.publish(sensor_data)

    def publishOccupancyGrid(self):
        grid = self.senseOccupancyGrid()

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom'

        map_meta_data = MapMetaData()
        map_meta_data.map_load_time = rospy.Time.now()
        map_meta_data.resolution = grid_resolution  # each cell is 15cm
        map_meta_data.width = int(grid_size / grid_resolution)
        map_meta_data.height = int(grid_size / grid_resolution)
        map_meta_data.origin = Pose(Point(0, 0, 0),
                                    Quaternion(0, 0, 0, 1))

        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info = map_meta_data
        grid_msg.data = list(grid)

        self.occupancyGridPublisher.publish(grid_msg)

    """Simulate a camera detecting a putting obstacles into a grid, with noise"""
    def senseOccupancyGrid(self):
        size = int(grid_size / grid_resolution)
        grid = np.zeros((size, size), dtype='int8')
        x, y, angle = self.robot.state[:, 0]

        for obs in self.obstacles:
            obs = obs - [x, y]  # translate to robot reference frame
            obs = self.rot_matrix(-angle).dot(obs)  # rotate to robot's reference frame

            obs = obs - self.camera_offset[:2]  # translate to camera frame
            obs = self.rot_matrix(-self.camera_offset[2]).dot(obs)  # rotate to camera's frame

            obs = obs / grid_resolution  # convert to grid cells

            # flip and offset because camera is centered on the bottom middle of the grid (add 0.5 because between grid)
            obs = [int(np.round(size - obs[1])), int(np.round(size/2 + obs[0]))]

            if 0 <= obs[0] < size and 0 <= obs[1] < size:  # add obstacle
                for i in range(int(obs[0]) - 1, int(obs[0]+1)):
                    for j in range(int(obs[1]) - 1, int(obs[1]+1)):
                        grid[i, j] = 50 + np.random.random() * 50

            for i, j in zip(np.random.randint(0, size, 00), np.random.randint(0, size, 0)):
                grid[i, j] = 50 + np.random.random() * 0  # random noise

        return grid.ravel()

    @staticmethod
    def rot_matrix(angle):
        return np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]])


if __name__ == "__main__":
    try:
        simulator = Simulator()
    except rospy.ROSInterruptException:
        pass
