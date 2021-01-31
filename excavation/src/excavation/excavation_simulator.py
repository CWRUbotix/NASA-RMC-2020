#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from hwctrl.msg import MotorCmd


class ExcavationSimulator:
    def __init__(self):
        rospy.init_node("excavation_simulator_node")
        rospy.loginfo("Excavation simulator node initialized")

        self.dumper_pos = 0  # m
        self.dumper_cmd_vel = 0  # speed
        self.dumper_top_sensor = False
        self.dumper_weight_sensor = 0  # (kg) weight reported by sensor
        self.dumper_weight = 0  # (kg) actual weight in bucket

        self.dumper_max_pos = 1.2  # m
        self.dumper_weight_start = 0.1
        self.dumper_weight_scale = 2
        self.dumper_encoder_scale = 1

        self.excavation_depth = 0  # m
        self.excavation_angle = 0  # rad
        self.excavation_depth_cmd = 0  # m
        self.excavation_angle_cmd = 0  # rad
        self.excavation_conveyor_cmd = 0  # speed

        rospy.Subscriber("dumper/motor_cmd", MotorCmd, self.receive_dumper_motor_cmd, queue_size=2)
        rospy.Subscriber("excavation/angle_cmd", MotorCmd, self.receive_excavation_angle_cmd, queue_size=2)
        rospy.Subscriber("excavation/depth_cmd", MotorCmd, self.receive_excavation_depth_cmd, queue_size=2)
        rospy.Subscriber("excavation/conveyor_cmd", MotorCmd, self.receive_excavation_conveyor_cmd, queue_size=2)

        self.dumper_top_limit_switch_pub = rospy.Publisher("dumper/top_limit_switch", Bool, queue_size=4)
        self.dumper_weight_pub = rospy.Publisher("dumper/weight", Float32, queue_size=4)
        self.dumper_position_pub = rospy.Publisher("dumper/position", Float32, queue_size=4)
        self.excavation_angle_pub = rospy.Publisher("excavation/angle", Float32, queue_size=4)
        self.excavation_depth_pub = rospy.Publisher("excavation/depth", Float32, queue_size=4)

        self.simulate_loop()

        rospy.spin()

    def simulate_loop(self):
        frequency = 15
        rate = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            # Dumper position controlled by velocity
            self.dumper_pos += self.dumper_cmd_vel * 1 / frequency
            self.dumper_pos = min(self.dumper_max_pos, max(0, self.dumper_pos))

            # When near the top the sensor triggers
            self.dumper_top_sensor = self.dumper_pos > self.dumper_max_pos - 0.1

            # Dumper weight increases if conveyor is putting dirt in it
            # When the bucket reaches the top the dirt falls out
            self.dumper_weight += 0.03 * self.excavation_conveyor_cmd
            if self.dumper_top_sensor:
                self.dumper_weight = 0

            # When dumper is close to the bottom that adds weight on the sensor
            # When the dumper raises the sensor no longer detects anything
            self.dumper_weight_sensor = max(0, self.dumper_weight_start - self.dumper_pos) * self.dumper_weight_scale
            self.dumper_weight_sensor += self.dumper_weight
            if self.dumper_pos > 2 * self.dumper_weight_start:
                self.dumper_weight_sensor = 0

            self.excavation_depth += 0.08 * (self.excavation_depth_cmd - self.excavation_depth)
            self.excavation_angle += 0.08 * (self.excavation_angle_cmd - self.excavation_angle)

            self.publish_sensor_data()

            try:
                rate.sleep()  # Wait for desired time
            except rospy.ROSInterruptException as e:
                rospy.loginfo(str(e))

    def publish_sensor_data(self):
        encoder_data = self.dumper_pos * self.dumper_encoder_scale

        self.dumper_top_limit_switch_pub.publish(self.dumper_top_sensor)
        self.dumper_weight_pub.publish(self.dumper_weight_sensor)
        self.dumper_position_pub.publish(encoder_data)
        self.excavation_angle_pub.publish(self.excavation_angle)
        self.excavation_depth_pub.publish(self.excavation_depth)

    def receive_dumper_motor_cmd(self, msg):
        self.dumper_cmd_vel = msg.setpoint

    def receive_excavation_angle_cmd(self, msg):
        self.excavation_angle_cmd = msg.setpoint

    def receive_excavation_depth_cmd(self, msg):
        self.excavation_depth_cmd = msg.setpoint

    def receive_excavation_conveyor_cmd(self, msg):
        self.excavation_conveyor_cmd = msg.setpoint
