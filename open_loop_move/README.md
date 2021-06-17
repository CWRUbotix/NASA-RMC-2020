#  Package: open_loop_move

Allows the robot to turn in place or drive straight without needing to follow a path.
Useful when you don't need too much accuracy.
Uses trapezoidal motion profiles. Will work with open loop,
but can use feedback by setting kP, kV, and kA to non-0.

## Nodes

### open_loop_move
Recieves request to have the robot drive straight or turn in place, and outputs cmd_vel to do so

#### Published Topics
* `cmd_vel` (`geometry_msgs/Twist`)
    The linear and angular velocity commands for the robot base
* `profile` (`geometry_msgs/Point`)
    The profile the robot is following, optionally published

#### Subscribed Topics
* `odom` (`nav_msgs/Odometry`)
    Robot odometry used in feedback, if desired.

#### Services
* `move_open_loop` (`open_loop_move/OpenLoopMove`)
    Used to trigger the move

#### Parameters
* `~dt` (`double`, default: `20.0`)
    The rate in Hz to run the control loop to send velocity commands
* `~max_vel_linear` (`double`, default: `0.5`)
    Maximum linear velocity
* `~max_accel_linear` (`double`, default: `1.5`)
    Maximum linear acceleration
* `~max_vel_angular` (`double`, default: `1.5`)
    Maximum angular velocity
* `~max_accel_angular` (`double`, default: `3.0`)
    Maximum angular acceleration
* `~kP_linear` (`double`, default: `0.0`)
    Proportional gain for position controller for linear displacement
* `~kP_angular` (`double`, default: `0.0`)
    Proportional gain for position controller for angular displacement
* `~kV_linear` (`double`, default: `0.0`)
    Proportional gain for velocity controller for linear displacement
* `~kV_angular` (`double`, default: `0.0`)
    Proportional gain for velocity controller for angular displacement
* `~kA_linear` (`double`, default: `0.0`)
    Proportional gain for acceleration controller for linear displacement
* `~kA_angular` (`double`, default: `0.0`)
    Proportional gain for acceleration controller for angular displacement
