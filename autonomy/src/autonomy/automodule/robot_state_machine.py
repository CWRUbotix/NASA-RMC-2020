#!/usr/bin/env python2

import rospy
import smach
import smach_ros


class Calibrate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['direction_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Calibrate')
        userdata.direction_out = 'dig'
        return 'succeeded'


class Drive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['driving', 'succeeded_dig', 'succeeded_dump'],
                             input_keys=['direction_in'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DRIVE')

        if self.counter < 3:
            self.counter += 1
            return 'driving'
        else:
            return 'succeeded_dig' if userdata.direction_in == 'dig' else 'succeeded_dump'


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

            smach.StateMachine.add('DRIVE', Drive(),
                                   transitions={'driving': 'DRIVE',
                                                'succeeded_dig': 'DIG',
                                                'succeeded_dump': 'DUMP'},
                                   remapping={'direction_in': 'sm_drive_direction'})

            smach.StateMachine.add('DIG', Dig(),
                                   transitions={'succeeded': 'DRIVE'},
                                   remapping={'direction_out': 'sm_drive_direction'})

            smach.StateMachine.add('DUMP', Dump(),
                                   transitions={'succeeded': 'succeeded'})

        return sm

    def shutdown(self):
        rospy.logwarn("shutting down robot state machine")
        self.sm.request_preempt()
