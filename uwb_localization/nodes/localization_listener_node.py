#!/usr/bin/env python3

import rospy

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

from hwctrl.msg import UwbData

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point

from localization.geoInterface import Target, Anchor
import localization.geoProject as Project

from uwb_localization.triangulation import UltraWideBandNode

class UwbLocalizationNode:

    node_topic = "localization_data"
    viz_dir = "visualizations/"


    def __init__(self, visualize=False):
        rospy.init_node("localization_listener", anonymous=False)

        self.robot_x = []
        self.robot_y = []
        self.robot_theta = []

        self.nodes = {}
        self.anchors = []

        self.pub = rospy.Publisher("uwb_nodes", PoseWithCovarianceStamped, queue_size=12)
        self.sub = rospy.Subscriber("localization_data", UwbData, self.position_callback, None, 12)
        
        # get anchors 
        for i in range(3):
            base_path = f"/hardware/anchors/anchor_{i}/"
            id =  int(rospy.get_param(base_path + "id"))
            x = float(rospy.get_param(base_path + "x"))
            y = float(rospy.get_param(base_path + "y"))
            self.anchors.append((id, x , y))
        
        # get uwbs
        for i in range(4):
            base_path = f"/hardware/sensors/uwb_{i}/"
            id = int(rospy.get_param(base_path + "id"))
            rel_x = int(rospy.get_param(base_path + "relative_x"))
            rel_y = int(rospy.get_param(base_path + "relative_x"))

            uwb = UltraWideBandNode(id, rel_x, rel_y, anchors)
            nodes[id] = uwb


    def get_robot_rotation(self):
        thetas = []
        node_pairs = combinations(self.nodes.values(), 2)
        for (start_node, end_node) in node_pairs:
            if start_node.is_valid() and end_node.is_valid():
                robot_edge = -np.array([start_node.relative_x, start_node.relative_y]) + np.array([end_node.relative_x, end_node.relative_y])

                # each edge vector is at a different angle relative to the robot coordinate system
                theta_offset = np.arctan2(robot_edge[1], robot_edge[0])
                dY = end_node.y - start_node.y
                dX = end_node.x - start_node.x
                theta = np.arctan2(dY, dX) - theta_offset
                thetas.append([np.cos(theta), np.sin(theta)])

        if len(thetas) > 0:
            theta = np.mean(thetas, axis=0)
            self.robot_theta.append(np.arctan2(theta[1], theta[0]))
        return 0 if len(self.robot_theta) == 0 else self.robot_theta[-1]


    def data_callback(self, msg):
        self.nodes[msg.node_id].add_measurement(msg.anchor_id, msg.distance, 0)
        self.nodes[msg.node_id].get_position()

        avg_x = 0
        avg_y = 0
        theta = self.get_robot_rotation()

        node_pairs = combinations(self.nodes.values(), 2)
        for (start_node, end_node) in node_pairs:
            if start_node.is_valid() and end_node.is_valid():
                # no idea what this line means? opposite corners?
                if start_node.relative_x == -end_node.relative_x and start_node.relative_y == -end_node.relative_y:
                    start_node.robot_x = (start_node.x + end_node.x) * 0.5
                    end_node.robot_x = start_node.robot_x
                    start_node.robot_y = (start_node.y + end_node.y) * 0.5
                    end_node.robot_y = start_node.robot_y
                    avg_x += start_node.robot_x
                    avg_y += start_node.robot_y
                    total += 1

        print('Total:', total)
        if total > 0:
            self.robot_x.append(avg_x / total)
            self.robot_y.append(avg_y / total)
            print('X: %.3f, Y: %.3f, theta: %.3f' % (self.robot_x[-1], self.robot_y[-1], theta))
            self.compose_msg()

    def compose_msg(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        point_msg = Point(self.robot_x[-1], self.robot_y[-1], 0)  # use most recent pos with no z-coord
        orientation_quat = R.from_euler('xyz', [0, 0, self.robot_theta[-1]]).as_quat()  # pitch is rotation about z-axis in euler angles
        pose_cov = np.diag([0.05, 0.05, 0, 0, 0, 0.01]).flatten()
        quat_msg = Quaternion(orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = Pose(point_msg, quat_msg)
        pose_with_cov.covariance = pose_cov
        stamped_msg = PoseWithCovarianceStamped()
        stamped_msg.header = header
        stamped_msg.pose = pose_with_cov
        try:
            self.pub.publish(stamped_msg)
        except rospy.ROSInterruptException as e:
            print(e.getMessage())
            pass


if __name__ == "__main__":
    node = UwbLocalizationNode()
#  uwb_sub = rospy.Subscriber(node.node_topic, UwbData, node.data_callback, None, 12)
    rospy.spin()
