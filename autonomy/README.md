#  Package: autonomy

The autonomy package is in charge of path planning, path following, and the overall robot state machine.
In the future these may be split into different packages

## Nodes

### autonomy_node.py
Currently, only opens a terminal to enter an x and y position for the robot to drive to.
Will soon become obsolete as this functionality will be in the client gui

### cmd_to_wheel_speeds_node.py
Takes in a cmd_vel topic, converts the linear and angular components into left and right wheel speeds.
Could alternatively be implemented in the hwctrl package.

#### Published Topics
* `/glenn_base/motor_cmds` (`hwctrl/DriveMotorCmd`)
    The speed commands to each side of the drive train

#### Subscribed Topics
* `/glenn_base/cmd_vel` (`geometry_msgs/Twist`)
    The linear and angular velocity commands for the robot base

#### Parameters
* todo

### robot_state_machine_node.py

#### Published Topics

#### Subscribed Topics


### simulator_node.py
Simulates a robot moving when given input commands and sending sensor data back. It May be useful for simple testing, but we also have the Gazebo simulation.

#### Publish topics  
* odometry/filtered_map
* motor_data
* local_occupancy_grid
#### Subscribed Topics
* motor_setpoints

### transit_node.py
Path planner and follower. Receives robot odometry, occupancy grid, goal position, and outputs velocity commands to drive to target position. 
#### Publish topics 
* /glenn_base/cmd_vel
* transit_path
* transit_control_data
#### Subscribed Topics
* glenn_base/odom_map
* global_occupancy_grid

### transit_test_module.py
A custom two dimensional robot simulator. Useful for testing path planning algorithms
