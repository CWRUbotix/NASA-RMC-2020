#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from hwctrl.msg import SetMotorMsg


class ExcavationSimulator:
    def __init__(self):
        rospy.init_node("excavation_simulator_node")
        rospy.loginfo("Excavation simulator node initialized")

        self.dumper_pos = 0
        self.dumper_cmd_vel = 0
        self.dumper_top_sensor = False
        self.dumper_weight_sensor = 0

        self.dumper_max_pos = 1.5
        self.dumper_weight_start = 0.1
        self.dumper_weight_scale = 2
        self.dumper_encoder_scale = 1

        rospy.Subscriber("dumper/motor_cmd", SetMotorMsg, self.receive_motor_msg, queue_size=2)
        self.dumper_top_limit_switch_pub = rospy.Publisher("dumper/top_limit_switch", Bool, queue_size=4)
        self.dumper_weight_pub = rospy.Publisher("dumper/weight", Float32, queue_size=4)
        self.dumper_position_pub = rospy.Publisher("dumper/position", Float32, queue_size=4)

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

            # Wen dumper is close to the bottom that puts weight on the sensor
            self.dumper_weight_sensor = max(0, self.dumper_weight_start - self.dumper_pos) * self.dumper_weight_scale

            self.publish_sensor_data()

            rate.sleep()

    def publish_sensor_data(self):
        encoder_data = self.dumper_pos * self.dumper_encoder_scale

        self.dumper_top_limit_switch_pub.publish(self.dumper_top_sensor)
        self.dumper_weight_pub.publish(self.dumper_weight_sensor)
        self.dumper_position_pub.publish(encoder_data)

    def receive_motor_msg(self, msg):
        self.dumper_cmd_vel = msg.setpoint
