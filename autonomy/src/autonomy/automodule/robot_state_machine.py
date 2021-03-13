#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from actionlib.msg import TestAction, TestGoal
from actionlib_msgs.msg import GoalStatus

from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal 


class Calibrate(smach.State):
    def __init__(self): smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['direction_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Calibrate')
        userdata.direction_out = 'dig'
        return 'succeeded'


class RobotStateMachine():
    def __init__(self):
        rospy.init_node("robot_state_machine")

        rospy.on_shutdown(self.shutdown)

        self.sm = self.construct_state_machine()

        sis = smach_ros.IntrospectionServer('robot_state_machine_server', self.sm, '/SM_ROOT')
        sis.start()

        self.outcome = self.sm.execute()

        rospy.spin()
        sis.stop()

    def construct_state_machine(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        sm.userdata.sm_drive_direction = 'none'
        sm.userdata.sub_run = 0

        # Create state machine
        with sm:
            smach.StateMachine.add('CALIBRATE', Calibrate(),
                                   transitions={'succeeded': 'DRIVE'},
                                   remapping={'direction_out': 'sm_drive_direction'})

            smach.StateMachine.add('DRIVE',
                                    smach_ros.SimpleActionState(
                                        'move_base',
                                        MoveBaseAction,
                                        goal_cb=self.drive_goal_cb,
                                        result_cb=self.drive_result_cb,
                                        input_keys=['direction_in', 'sub_run'],
                                        outcomes=['succeeded_dig', 'succeeded_dump']
                                    ),
                                   transitions={'succeeded_dig': 'DIG',
                                                'succeeded_dump': 'DUMP'},
                                   remapping={'direction_in': 'sm_drive_direction'})

            smach.StateMachine.add('DIG',
                                    smach_ros.SimpleActionState(
                                        'dig',
                                        TestAction,
                                        goal=TestGoal(0),
                                        result_cb=self.dig_result_cb,
                                        outcomes=['succeeded'],
                                        output_keys=['direction_out']
                                    ),
                                    transitions={'succeeded': 'DRIVE'},
                                    remapping={'direction_out': 'sm_drive_direction'})

            smach.StateMachine.add('DUMP',
                                    smach_ros.SimpleActionState(
                                        'dump',
                                        TestAction,
                                        goal=TestGoal(1),
                                        result_cb=self.dump_result_cb,
                                        input_keys=['sub_run'],
                                        outcomes=['succeeded'],
                                    ),
                                   transitions={'succeeded': 'succeeded'})

        return sm

    def drive_goal_cb(self, userdata, goal):
        drive_goal = MoveBaseActionGoal()

        drive_goal.goal.target_pose.header.frame_id = "map"
        drive_goal.goal.target_pose.pose.orientation.w = 1

        dig_spots = [(1, 5.75), (1, 4.75), (1.75, 5.75), (1.75, 4.75)]
        bin_spot = (1.5, 0.85)

        position = drive_goal.goal.target_pose.pose.position

        if userdata.direction_in == "dig":
            position.x = dig_spots[userdata.sub_run][0]
            position.y = dig_spots[userdata.sub_run][1]
        else:
            position.x = bin_spot[0]
            position.y = bin_spot[1]

        rospy.loginfo("State DRIVE sending goal: " + str(position.x) + ", " + str(position.y))

        return drive_goal.goal

    def drive_result_cb(self, userdata, status, result):
        rospy.loginfo("DRIVE state returned with status: " + GoalStatus.to_string(status))
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded_dig' if userdata.direction_in == 'dig' else 'succeeded_dump'

    def dig_result_cb(self, userdata, status, result):
        rospy.loginfo("DIG state returned with status: " + GoalStatus.to_string(status))
        userdata.direction_out = 'dump'
        return 'succeeded'

    def dump_result_cb(self, userdata, status, result):
        rospy.loginfo("DUMP state returned with status: " + GoalStatus.to_string(status))
        userdata.sub_run += 1
        return 'succeeded'

    def shutdown(self):
        rospy.logwarn("Shutting down robot state machine")
        self.sm.request_preempt()
