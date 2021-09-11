# cmd_to_wheel_speeds_node

Takes in a cmd_vel topic, converts the linear and angular components into left and right wheel speeds.
Could alternatively be implemented in the hwctrl package.

## API

### Subscribed Topics

* `/glenn_base/cmd_vel` (`geometry_msgs/Twist`)
    * The linear and angular velocity commands for the robot base

### Published Topics

* `/glenn_base/motor_cmds` (`hwctrl/DriveMotorCmd`)
  * The speed commands to each side of the drive train

### Parameters
* TODO