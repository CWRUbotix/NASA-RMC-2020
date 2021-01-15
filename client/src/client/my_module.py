#!/usr/bin/env python
import os

import rospy, rospkg, sys, rosservice, robotInterface
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from actionlib.msg import TestAction
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from hwctrl.msg import Encoders


node_name = 'robot_interface'
motorCommandTopic = 'motor_setpoints'
sensorValueTopic = 'sensor_value'

# --------------------------------------------------
#
# Heavily referenced from rosviz - rqt_robot_steering
# For CWRUbotix
#
# Manual Control Subteam
# ---------------------------------------------------

class MyPlugin(Plugin):
    
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
       
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()

        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
       
        args, unknowns = parser.parse_known_args(context.argv())
       
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('client'), 'resource', 'MyPlugin.ui')
       
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        #Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.setFocusPolicy(0x8)
        self._widget.setFocus()

        #Widget vars
        self.ENABLE_DEBUGGING = False
        self.general_speed_slider_conversion_factor = 0.1
        self.excavation_angle_slider_conversion_factor = 0.01
        self._subscriber_widget_map = {}

        self._widget.stop_updating_button.setEnabled(False)
       
        '''Set reasonable ranges for the motors
         deposition bucket speed is from -1 to 1'''
        self._widget.dep_bucket_speed_spinbox.setRange(-1, 1)
        self._widget.dep_bucket_speed_spinbox.setSingleStep(0.1)

        '''excavation speed is from -1 to 1'''
        self._widget.excavation_speed_spinbox.setRange(-1, 1)
        self._widget.excavation_speed_spinbox.setSingleStep(0.1)

        '''excavation depth is from 0 to 0.4'''
        self._widget.excavation_depth_spinbox.setRange(0, 0.4)
        self._widget.excavation_depth_spinbox.setSingleStep(0.05)

        '''excavation angle is from 0 to 1.57'''
        self._widget.excavation_angle_spinbox.setRange(0, 1.57)
        self._widget.excavation_angle_spinbox.setSingleStep(0.02)

        #Configure the slider counterpart
        self._widget.excavation_angle_slider.setRange(0, 157)
        self._widget.excavation_angle_slider.setSingleStep(2)
        
        '''drive speed is from -0.6 to 0.6
            can also used to control all other motor spinbox values'''
        self._widget.general_speed_spinbox.setRange(0, 0.6)
        self._widget.general_speed_spinbox.setSingleStep(0.1)

        #Configure the slider counterpart 
        self._widget.general_speed_slider.setRange(0, 6)
        self._widget.general_speed_slider.setSingleStep(1)

        #Configure the goal value slider 
        self._widget.goal_value_slider.setRange(0, 2)
        self._widget.goal_value_slider.setSingleStep(1)

        #All spinbox valueChanged callbacks
        self._widget.dep_bucket_speed_spinbox.valueChanged.connect(self.dep_bucket_speed_changed)
        self._widget.excavation_speed_spinbox.valueChanged.connect(self.excavation_speed_changed)
        self._widget.excavation_depth_spinbox.valueChanged.connect(self.excavation_depth_changed)
        self._widget.excavation_angle_spinbox.valueChanged.connect(self.excavation_angle_changed)
        self._widget.general_speed_spinbox.valueChanged.connect(self.general_spinbox_changed)

        #All slider valueChanged callbacks
        self._widget.excavation_angle_slider.valueChanged.connect(self.excavation_angle_slider_changed)
        self._widget.goal_value_slider.valueChanged.connect(self.goal_value_slider_changed)
        self._widget.general_speed_slider.valueChanged.connect(self.general_slider_changed)

        #All button pressed callbacks
        self._widget.emergency_stop_button.pressed.connect(self.estop_pressed)
        self._widget.w_button.pressed.connect(self.w_pressed)
        self._widget.a_button.pressed.connect(self.a_pressed)
        self._widget.s_button.pressed.connect(self.s_pressed)
        self._widget.d_button.pressed.connect(self.d_pressed)
        self._widget.dig_button.pressed.connect(self.dig_pressed)
        self._widget.dump_button.pressed.connect(self.dump_pressed)
        self._widget.debug_checkbox.toggled.connect(self.debugging_checkbox_checked)
        self._widget.start_updating_button.pressed.connect(self.start_updating_sensors)
        self._widget.stop_updating_button.pressed.connect(self.stop_updating_sensors)

        ### Keyboard teleop setup
        self._widget.keyPressEvent = self.keyPressEvent
        self._widget.keyReleaseEvent = self.keyReleaseEvent
        
    def debugging_checkbox_checked(self):
        debugging_status = self._widget.debug_checkbox.isChecked()
        self.ENABLE_DEBUGGING = debugging_status

    # Keyboard Teleop with signalling
    """
    Messy, but it works, theoretically.
    """
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            self.w_pressed()
        elif event.key() == Qt.Key_S:
            self.s_pressed()
        elif event.key() == Qt.Key_A:
            self.a_pressed()
        elif event.key() == Qt.Key_D:
            self.d_pressed()
        # Emergency stop, triggers for all motors. Can include one explicitly defined for locomotion though
        elif event.key() == Qt.Key_E:
            rospy.loginfo("Emergency Stopping")
            self.estop_pressed()
        
        # Deposition keys
        elif event.key() == Qt.Key_U:
            rospy.loginfo("Press U key")

        # Arrow keys to manipulate the general spinbox speed
        elif event.key() == Qt.Key_Up:
            motor_speed = self.get_general_motor_val()
            rospy.loginfo("Key up")
            self._widget.general_speed_spinbox.setValue(motor_speed + 1)

        elif event.key() == Qt.Key_Down:
            motor_speed = self.get_general_motor_val()
            rospy.loginfo("Key down")
            self._widget.general_speed_spinbox.setValue(motor_speed - 1)

    # Currently only geared toward locomotion
    def keyReleaseEvent(self, event):
        if event.key() in (Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D):
            if not event.isAutoRepeat():
                rospy.loginfo("Key released")
                robotInterface.sendDriveCommand(0, 0)
        
    def get_general_motor_val(self):
        val = float(self._widget.general_speed_spinbox.value())
        return val

    def w_pressed(self):
        motor_speed = self.get_general_motor_val()
        robotInterface.sendDriveCommand(0, motor_speed)
        if self.ENABLE_DEBUGGING:
            rospy.loginfo("w key pressed")

    def a_pressed(self):
        motor_speed = self.get_general_motor_val()
        robotInterface.sendDriveCommand(3, motor_speed)
        if self.ENABLE_DEBUGGING:
            rospy.loginfo("a key pressed")

    def s_pressed(self):
        motor_speed = self.get_general_motor_val()
        robotInterface.sendDriveCommand(1, motor_speed)
        if self.ENABLE_DEBUGGING:
            rospy.loginfo("s key pressed")

    def d_pressed(self):
        motor_speed = self.get_general_motor_val()
        robotInterface.sendDriveCommand(2, motor_speed)
        if self.ENABLE_DEBUGGING:
            rospy.loginfo("d key pressed")

    def estop_pressed(self):
        rospy.loginfo("ESTOP: Attempting to stop all motors...")
    
        # Set all known motors to value 0
        robotInterface.sendWheelSpeed(0)
        robotInterface.sendExcavationSpeed(0)
        robotInterface.sendDepositionBucketSpeed(0)

        # Excavation depth and conveyor angle are set to the most recent position from SensorValues
        exc_depth = self._widget.excavation_depth_sv.text()
        exc_angle = self._widget.excavation_depth_sv.text()

        if exc_depth == '': exc_depth = 0.0
        if exc_angle == '': exc_angle = 0.0

        robotInterface.sendExcavationDepth(float(exc_depth))
        robotInterface.sendExcavationAngle(float(exc_angle))

    def dig_pressed(self):
        val = int(self._widget.goal_value_slider.value())
        robotInterface.sendDigAction(val)

    def dump_pressed(self):
        val = int(self._widget.goal_value_slider.value())
        robotInterface.sendDumpAction(val)

    """
    Unregister ROS publisher
    """
    def _unregister_publisher(self):
       self.stop_updating_sensors()


    """
    Individual Motor Change Functions
    """
    def dep_bucket_speed_changed(self):
        val = float(self._widget.dep_bucket_speed_spinbox.value())
        robotInterface.sendDepositionBucketSpeed(val)

    def excavation_speed_changed(self):
        val = float(self._widget.excavation_speed_spinbox.value())
        robotInterface.sendExcavationSpeed(val)

    def excavation_depth_changed(self):
        val = float(self._widget.excavation_depth_spinbox.value())
        robotInterface.sendExcavationDepth(val)

    def excavation_angle_changed(self):
        val = float(self._widget.excavation_angle_spinbox.value())
        robotInterface.sendExcavationAngle(val)
        self._widget.excavation_angle_slider.setValue(int(val * 100))

    def excavation_angle_slider_changed(self):
        val = float (self._widget.excavation_angle_slider.value() * self.excavation_angle_slider_conversion_factor)
        self._widget.excavation_angle_spinbox.setValue(val)

    def goal_value_slider_changed(self):
        val = self._widget.goal_value_slider.value()
        self._widget.goal_value_label.setText(str(val))

    """
    Grouped Motor Control Functions
    """
    def general_spinbox_changed(self):
        self._widget.general_speed_slider.setValue(int(self.get_general_motor_val() * 10))
        if self._widget.general_assign_checkbox.isChecked():
            motor_speed = self.get_general_motor_val()
            self._widget.dep_bucket_speed_spinbox.setValue(motor_speed)
            self._widget.excavation_speed_spinbox.setValue(motor_speed)
            self._widget.excavation_depth_spinbox.setValue(motor_speed)
            self._widget.excavation_angle_spinbox.setValue(motor_speed)

    def general_slider_changed(self):
        sliderValue = int(self._widget.general_speed_slider.value())
        self._widget.general_speed_spinbox.setValue(sliderValue * self.general_speed_slider_conversion_factor)    

    """
    Sensor value updating 
    """
    def start_updating_sensors(self):

        self._widget.stop_updating_button.setEnabled(True)
        self._widget.start_updating_button.setEnabled(False)

        #Initialize the subscribers
        self._imu_data_sub = rospy.Subscriber('/imu/data_raw', Imu, lambda data: self._widget.imu_data_raw_sv.setText(str(data.angular_velocity.z)))
        self._realsense_data_sub = rospy.Subscriber('/realsense/imu/data_raw', Imu, lambda data: self._widget.realsense_data_raw_sv.setText(str(data.angular_velocity.z)))
        self._top_limit_switch_sub = rospy.Subscriber('/dumper/top_limit_switch', Bool, lambda data: self._widget.top_limit_switch_sv.setText(str(data.data)))
        self._dumper_position_sub = rospy.Subscriber('/dumper/position', Float32, lambda data: self._widget.dumper_position_sv.setText(str(data.data)))
        self._dumper_weight_sub = rospy.Subscriber('/dumper/weight', Float32, lambda data: self._widget.dumper_weight_sv.setText(str(data.data)))
        self._exc_depth_sub = rospy.Subscriber('/excavation/depth', Float32,lambda data: self._widget.excavation_depth_sv.setText(str(data.data)))
        self._exc_angle_sub = rospy.Subscriber('/excavation/angle', Float32,lambda data: self._widget.excavation_angle_sv.setText(str(data.data)))
        self._encocoders_sub = rospy.Subscriber('/glenn_base/encoders', Encoders,lambda data: self._widget.encoders_sv.setText('Left: %.3f, right:%.3f'%(data.left, data.right)))

        #Link subscribers to their widgets
        self._subscriber_widget_map = {
            self._imu_data_sub:self._widget.imu_data_raw_sv,
            self._realsense_data_sub:self._widget.realsense_data_raw_sv,
            self._top_limit_switch_sub:self._widget.top_limit_switch_sv,
            self._dumper_position_sub:self._widget.dumper_position_sv,
            self._dumper_weight_sub:self._widget.dumper_weight_sv,
            self._exc_depth_sub:self._widget.excavation_depth_sv,
            self._exc_angle_sub:self._widget.excavation_angle_sv,
            self._encocoders_sub:self._widget.encoders_sv
        }

    def stop_updating_sensors(self):
        self._widget.stop_updating_button.setEnabled(False)
        self._widget.start_updating_button.setEnabled(True)

        for subscriber, sv_widget in self._subscriber_widget_map.items():
            subscriber.unregister()
            sv_widget.setText('')

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        #self._update_parameter_timer.stop()
        self._unregister_publisher()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
