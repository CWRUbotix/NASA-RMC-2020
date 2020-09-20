#!/usr/bin/env python2

import rospy
import smach
import smach_ros
from actionlib_msgs.msg import GoalStatus

from glenn_msgs.msg import GoToGoalAction, GoToGoalGoal


class Calibrate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['direction_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Calibrate')
        userdata.direction_out = 'dig'
        return 'succeeded'


class Dig(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['direction_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DIG')
        userdata.direction_out = 'dump'
        return 'succeeded'


class Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DUMP')
        return 'succeeded'


class RobotStateMachine():
    def __init__(self):
        rospy.init_node("robot_state_machine")

        rospy.on_shutdown(self.shutdown)

        self.sm = self.construct_state_machine()

        sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
        sis.start()

        self.outcome = self.sm.execute()

        rospy.spin()
        sis.stop()

    def construct_state_machine(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        sm.userdata.sm_drive_direction = 'none'

        # Create state machine
        with sm:
            smach.StateMachine.add('CALIBRATE', Calibrate(),
                                   transitions={'succeeded': 'DRIVE'},
                                   remapping={'direction_out': 'sm_drive_direction'})

            smach.StateMachine.add('DRIVE',
                                    smach_ros.SimpleActionState(
                                        'go_to_goal',
                                        GoToGoalAction,
                                        goal_cb=self.drive_goal_cb,
                                        result_cb=self.drive_result_cb,
                                        input_keys=['direction_in'],
                                        outcomes=['succeeded_dig', 'succeeded_dump']),
                                   transitions={'succeeded_dig': 'DIG',
                                                'succeeded_dump': 'DUMP'},
                                   remapping={'direction_in': 'sm_drive_direction'})

            smach.StateMachine.add('DIG', Dig(),
                                   transitions={'succeeded': 'DRIVE'},
                                   remapping={'direction_out': 'sm_drive_direction'})

            smach.StateMachine.add('DUMP', Dump(),
                                   transitions={'succeeded': 'succeeded'})

        return sm

    def drive_goal_cb(self, userdata, goal):
        if userdata.direction_in == 'dig':
            drive_goal = GoToGoalGoal(x=2.5, y=1.5)
        else:
            drive_goal = GoToGoalGoal(x=1.5, y=0.75)

        rospy.loginfo("State DRIVE sending goal: " + str(drive_goal.x) + ", " + str(drive_goal.y))
        return drive_goal

    def drive_result_cb(self, userdata, status, result):
        rospy.loginfo("DRIVE state returned with status: " + GoalStatus.to_string(status))
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded_dig' if userdata.direction_in == 'dig' else 'succeeded_dump'

    def shutdown(self):
        rospy.logwarn("Shutting down robot state machine")
        self.sm.request_preempt()
