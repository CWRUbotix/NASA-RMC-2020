#!/usr/bin/env python
import os

import rospy
import rospkg
import sys
import rosservice

import robotInterface

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtWidgets import QWidget

node_name = 'robotInterface'
motorCommandTopic = 'motorCommand'
sensorValueTopic = 'sensorValue'

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
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # ROS Publisher
        self._publisher = None

        robotInterface.initializeRobotInterface()

        # Service Proxy and Subscriber
        self._service_proxy = None
        self._subscriber = None

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('client'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        
        ### Map motors to their respective UI elements
        self.motor_widgets = {
            0:self._widget.motor0_spinbox,
            1:self._widget.motor1_spinbox,
            2:self._widget.motor2_spinbox,
            3:self._widget.motor3_spinbox,
            4:self._widget.motor4_spinbox,
            5:self._widget.motor5_spinbox,
            6:self._widget.motor6_spinbox,
            7:self._widget.motor7_spinbox
        }

        self.sensor_widgets = {
            0:self._widget.sensor0_lineedit,
            1:self._widget.sensor1_lineedit,
            2:self._widget.sensor2_lineedit,
            3:self._widget.sensor3_lineedit,
            4:self._widget.sensor4_lineedit,
            5:self._widget.sensor5_lineedit,
            6:self._widget.sensor6_lineedit,
            7:self._widget.sensor7_lineedit,
            8:self._widget.sensor8_lineedit,
            9:self._widget.sensor9_lineedit,
            10:self._widget.sensor10_lineedit,
            13:self._widget.sensor13_lineedit,
            19:self._widget.sensor19_lineedit,
            23:self._widget.sensor23_lineedit,
            24:self._widget.sensor24_lineedit,
            25:self._widget.sensor25_lineedit,
            26:self._widget.sensor26_lineedit,
            27:self._widget.sensor27_lineedit,
            28:self._widget.sensor28_lineedit,
            29:self._widget.sensor29_lineedit,
            30:self._widget.sensor30_lineedit,
            31:self._widget.sensor31_lineedit,
            32:self._widget.sensor32_lineedit,
        }

        # Assigned zeros for values that indicate motor speed
        # Motors 4 and 5 are set to the most recent position from SensorValues
        self.zero_values = {
            0:0,
            1:0,
            2:0,
            3:0,
            6:0,
            7:0
        }
        #print(self.motor_widgets)
        # Hook Qt UI Elements up
        """
        self._widget.vertical_add_button.pressed.connect(self.increase_linear_speed_pressed)
        self._widget.vertical_subtract_button.pressed.connect(self.decrease_linear_speed_pressed)
        """
        self.ENABLE_DEBUGGING = False

        self._widget.motor0_spinbox.valueChanged.connect(self.motor0_spinbox_changed)
        self._widget.motor1_spinbox.valueChanged.connect(self.motor1_spinbox_changed)
        self._widget.motor2_spinbox.valueChanged.connect(self.motor2_spinbox_changed)
        self._widget.motor3_spinbox.valueChanged.connect(self.motor3_spinbox_changed)
        self._widget.motor4_spinbox.valueChanged.connect(self.motor4_spinbox_changed)
        self._widget.motor5_spinbox.valueChanged.connect(self.motor5_spinbox_changed)
        self._widget.motor6_spinbox.valueChanged.connect(self.motor6_spinbox_changed)
        self._widget.motor7_spinbox.valueChanged.connect(self.motor7_spinbox_changed)

        self._widget.general_speed_spinbox.valueChanged.connect(self.general_spinbox_changed)
        self._widget.general_speed_slider.valueChanged.connect(self.general_slider_changed)

        self._widget.emergency_stop_button.pressed.connect(self.estop_pressed)
        self._widget.w_button.pressed.connect(self.w_pressed)
        self._widget.a_button.pressed.connect(self.a_pressed)
        self._widget.s_button.pressed.connect(self.s_pressed)
        self._widget.d_button.pressed.connect(self.d_pressed)

        self._widget.zero_locomotion_button.pressed.connect(self.zero_locomotion_speeds)
        
        self._widget.start_shift_button.pressed.connect(self.setup_translate_shift_timer)
        self._widget.translate_cancel_button.pressed.connect(self.cancel_shift)
        self._widget.attitude_shift_button.pressed.connect(self.setup_attitude_timer)
        self._widget.attitude_shift_cancel_button.pressed.connect(self.cancel_attitude_shift)

        self._widget.update_sensors_button.pressed.connect(self._setup_timer_update_sensors)
        self._widget.stop_sensor_update_button.pressed.connect(self._stop_update_timer)

        self._widget.debug_checkbox.toggled.connect(self.debugging_checkbox_checked)

        # ROS Connection Fields
        """
        TODO: Omitted
        """
        ###

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.setFocusPolicy(0x8)
        self._widget.setFocus()

        ### Keyboard teleop setup
        self._widget.keyPressEvent = self.keyPressEvent
        self._widget.keyReleaseEvent = self.keyReleaseEvent

        # timer to consecutively send service messages
        self._update_translate_timer = QTimer(self)
        self._update_attitude_timer = QTimer(self)
        self._update_sensors_timer = QTimer(self)
        """
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False
        """
        
    def debugging_checkbox_checked(self):
        debugging_status = self._widget.debug_checkbox.isChecked()
        self.ENABLE_DEBUGGING = debugging_status

    # Keyboard Teleop with signalling
    """
    Messy, but it works, theoretically.
    """
    def keyPressEvent(self, event):
        motor_speed = self.get_general_motor_val()
        #print("general motor value is: %s" % motor_speed)
        if event.key() == Qt.Key_W:
            #print("W down")
            self.w_pressed(motor_speed)
        elif event.key() == Qt.Key_S:
            #print("S down")
            self.s_pressed(motor_speed)
        elif event.key() == Qt.Key_A:
            #print("A down")
            self.a_pressed(motor_speed)
        elif event.key() == Qt.Key_D:
            #print("D down")
            self.d_pressed(motor_speed)
        # Emergency stop, triggers for all motors. Can include one explicitly defined for locomotion though
        elif event.key() == Qt.Key_E:
            print("Emergency Stopping")
            self.estop_pressed()
        
        # Deposition keys
        elif event.key() == Qt.Key_U:
            print("Press U key")

        # Arrow keys to manipulate the general spinbox speed
        elif event.key() == Qt.Key_Up:
            print("Key up")
            self._widget.general_speed_spinbox.setValue(motor_speed + 1)

        elif event.key() == Qt.Key_Down:
            print("Key down")
            self._widget.general_speed_spinbox.setValue(motor_speed - 1)

    # Generalize sending spinbox value, handle print/error here
    def send_spinbox_value(self, motorNum, value):
        resp = robotInterface.sendMotorCommand(motorNum, value)
        print("For motor %s sent value %s successfully: %s" % (motorNum, value, resp))

    # Currently only geared toward locomotion

    def keyReleaseEvent(self, event):
        if event.key() in (Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D):
            if not event.isAutoRepeat():
                print("Key released")
                robotInterface.sendDriveCommand(0, 0)

    def set_locomotion_speeds(self, port_speed, starboard_speed):
        respPort = robotInterface.sendMotorCommand(0, port_speed)
        respStarboard = robotInterface.sendMotorCommand(1, starboard_speed)
        print("Set locomotion speeds: %s" % (respPort and respStarboard))

    def zero_locomotion_speeds(self):
        self.motor_widgets.get(0).setValue(0)
        self.motor_widgets.get(1).setValue(0)
        self.set_locomotion_speeds(0,0)
        
    def get_general_motor_val(self):
        val = int(self._widget.general_speed_spinbox.value())
        return val

    def w_pressed(self, motor_speed=None):
        if motor_speed is None:
            motor_speed = self.get_general_motor_val()
        robotInterface.sendDriveCommand(0, motor_speed)
        if self.ENABLE_DEBUGGING:
            print("w key pressed")

    def a_pressed(self, motor_speed=None):
        if motor_speed is None:
            motor_speed = self.get_general_motor_val()

        robotInterface.sendDriveCommand(3, motor_speed)
        if self.ENABLE_DEBUGGING:
            print("a key pressed")

    def s_pressed(self, motor_speed=None):
        if motor_speed is None:
            motor_speed = self.get_general_motor_val()
        robotInterface.sendDriveCommand(1, motor_speed)
        if self.ENABLE_DEBUGGING:
            print("s key pressed")

    def d_pressed(self, motor_speed=None):
        if motor_speed is None:
            motor_speed = self.get_general_motor_val()
        robotInterface.sendDriveCommand(2, motor_speed)
        if self.ENABLE_DEBUGGING:
            print("d key pressed")

    def estop_pressed(self):
        print("ESTOP: Attempting to stop all motors...")
        
        # Set all known motors to value 0
        for motor_id, zero_value in self.zero_values.items():
            #ui_widget.setValue(0)
            robotInterface.sendMotorCommand(motor_id, 0)

        # Motors 4 and 5 are set by position
        translationPos = robotInterface.sensorValueMap.get(4)
        bcAttitudePos = robotInterface.sensorValueMap.get(5)

        print("ESTOP: Setting translation position: %s and attitude: %s" % (translationPos, bcAttitudePos) )

        robotInterface.sendMotorCommand(4, translationPos)
        robotInterface.sendMotorCommand(5, bcAttitudePos)

        # Stop any updated changes to the translation system
        self._update_translate_timer = QTimer(self)

    # ROS Connection things
    """
    @Slot(str)
    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        print("trying topic: ", topic)
        if topic == '':
            return
        try:
            self._publisher = rospy.Publisher(topic, Twist, queue_size = 10)
        except TypeError:
            self._publisher = rospy.Publisher(topic, Twist)
    """

    """
    Unregister ROS publisher
    """
    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

        if self._service_proxy is not None:
            # TODO:Doesn't actually shutdown/unregister??
            #self._service_proxy.shutdown('Shutting down service proxy...')
            self._service_proxy = None

        #if robotInterface is not None:
        #    robotInterface.motorCommandPub.unregister()


    #### Speed and Angle change Functions

    """
    Individual Motor Change Functions
    """

    def motor0_spinbox_changed(self):
        val = int(self.motor_widgets.get(0).value())
        self.send_spinbox_value(0, val)

    def motor1_spinbox_changed(self):
        val = int(self.motor_widgets.get(1).value())
        self.send_spinbox_value(1, val)

    def motor2_spinbox_changed(self):
        val = int(self.motor_widgets.get(2).value())
        self.send_spinbox_value(2, val)

    def motor3_spinbox_changed(self):
        val = int(self.motor_widgets.get(3).value())
        self.send_spinbox_value(3, val)

    def motor4_spinbox_changed(self):
        val = int(self.motor_widgets.get(4).value())
        self.send_spinbox_value(4, val)

    def motor5_spinbox_changed(self):
        val = int(self.motor_widgets.get(5).value())
        self.send_spinbox_value(5, val)
        self._widget.attitude_slider.setValue(val)

    def motor5_slider_changed(self):
        val = int(self._widget.attitude_slider.value())

    ### Looky Spinboxes

    def motor6_spinbox_changed(self):
        val = int(self.motor_widgets.get(6).value())
        self.send_spinbox_value(6, val)
    def motor7_spinbox_changed(self):
        val = int(self.motor_widgets.get(7).value())
        self.send_spinbox_value(7, val)

    ### Translation timer-updated shift to some specified value
    def setup_translate_shift_timer(self):
        updatesPerSec = int(self._widget.steps_per_sec_spinbox.value())
        msUpdate = int(1000/updatesPerSec)
        self._update_translate_timer = QTimer()
        self._update_translate_timer.timeout.connect(self.shift_timer_func)
        self._update_translate_timer.start(msUpdate)

    def shift_timer_func(self):
        currentValue = self._widget.motor4_spinbox.value()
        targetValue = int(self._widget.target_translate_spinbox.value())
        if(targetValue == currentValue):
            self._update_translate_timer = QTimer(self)
            self._widget.translate_cancel_button.setEnabled(False)
            return
        else:
            delta = (targetValue - currentValue)/abs(targetValue - currentValue)
            #print(delta)
            self._widget.motor4_spinbox.setValue(currentValue + delta)
            self._widget.translate_cancel_button.setEnabled(True)

    def setup_attitude_timer(self):
        updatesPerSec = int(self._widget.steps_attitude_spinbox.value())
        msUpdate = int(1000/updatesPerSec)
        self._update_attitude_timer = QTimer()
        self._update_attitude_timer.timeout.connect(self.shift_timer_attitude_func)
        self._update_attitude_timer.start(msUpdate)

    def shift_timer_attitude_func(self):
        currentValue = self._widget.motor5_spinbox.value()
        targetValue = int(self._widget.target_attitude_spinbox.value())
        if(targetValue == currentValue):
            self._update_attitude_timer = QTimer(self)
            self._widget.attitude_shift_cancel_button.setEnabled(False)
        else:
            delta = (targetValue - currentValue)/abs(targetValue - currentValue)
            self._widget.motor5_spinbox.setValue(currentValue + delta)
            self._widget.attitude_shift_cancel_button.setEnabled(True)
    
    # Cancel the 'shift' by just setting the reference to a new QTimer
    def cancel_shift(self):
        self._update_translate_timer.stop()
        self._widget.translate_cancel_button.setEnabled(False)

    def cancel_attitude_shift(self):
        self._update_attitude_timer.stop()
        self._widget.attitude_shift_cancel_button.setEnabled(False)

    """
    Grouped Motor Control Functions
    """

    def general_spinbox_changed(self):
        self._widget.general_speed_slider.setValue(self.get_general_motor_val())
        if self._widget.general_assign_checkbox.isChecked():
            motor_speed = self.get_general_motor_val()
            for motor_id, ui_widget in self.motor_widgets.items():
                ui_widget.setValue(motor_speed)
        else:
            return

    def general_slider_changed(self):
        sliderValue = int(self._widget.general_speed_slider.value())
        self._widget.general_speed_spinbox.setValue(sliderValue)
        #print("Slider value is: %s" % sliderValue)
    
    """
    Update sensor values
    """
    

    """
     Sending messages
    """
    # Iteratively send values
    def _on_parameter_changed(self):
        for motor_id, ui_widget in self.motor_widgets.items():
            # If widget is spinbox,
            val = int(ui_widget.value())
            resp = self._send_motor_command(motor_id, val)
            print("on motor: %s" % motor_id + ", value: %s" % val + "sending result: %s" % resp)

    """
    Sends a single commanded value (0-100) to a specified motor id
    """
    def _send_motor_command(self, motor_id, val):
        try:
            robotInterface.sendMotorCommand(motor_id, val)
            print("Sent motor command for motor: %s" % motor_id + " with value: %s" % val)
        except Exception as exc:
            print("There was a problem sending the motor command: " + str(exc))

    """
    Sensor value updating 
    """
    def _stop_update_timer(self):
        self._update_sensors_timer = QTimer()

    def _setup_timer_update_sensors(self):
        updatesPerSec = 100
        msUpdate = int(1000/updatesPerSec)
        self._update_sensors_timer = QTimer()
        self._update_sensors_timer.timeout.connect(self.try_update_sensors)
        self._update_sensors_timer.start(msUpdate)

    def try_update_sensors(self):
        try:
            for sensor_id, sensor_widget in self.sensor_widgets.items():
                # Try to get value
                sensorVal = robotInterface.sensorValueMap.get(sensor_id)
                sensor_widget.setText(str(sensorVal))
        except Exception as exc:
            print("There was an unusual problem updating sensors: " + str(exc))


    def _set_status_text(self, text):
        self._widget.status_label.setText(text)

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
