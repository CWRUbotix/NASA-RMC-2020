#  Package: uwb_localization

Handles the ultra wide band localization algorithm to find robot's position and orientation
on the field.

Also has nodes to receive IMU and encoder messages and publishes open loop odometry from them

## Nodes

### localization_listener_node.py
Receives data from the uwb recievers and calulcates the likely position of the robot with least squares

#### Published Topics
* `uwb_nodes` (`geometry_msgs/PoseWithCovarianceStamped`)
    The calculated pose from the uwb system

#### Subscribed Topics
* `localization_data` (`hwctrl/UwbData`)
    The raw data from the uwb recievers


### imu_listener_node
Receives data from the imu and wheel encoders and calculates odometry

#### Published Topics
* `wheel` (`nav_msgs/Odometry`)
    Raw robot odometry from just encoders
* `imu/data` (`sensor_msgs/Imu`)
    Republished the imu data out corrected for bias

#### Subscribed Topics
* `imu/data_raw` (`sensor_msgs/Imu`)
    The raw data from the imu
* `glenn_base/encoders` (`hwctrl/Encoders`)
    Raw encoder data for both sides of the robot

#### Parameters
* `wheel_radius` (`double`, default: `N/A`)
    Robot wheel radius, used for odometry
* `effective_robot_width` (`double`, default: `N/A`)
    Effective robot wheel base accounting for slip, used for odom
* `imu_angular_vel_x_offset` (`double`, default: `N/A`)
    Angular velocity bias x
* `imu_angular_vel_y_offset` (`double`, default: `N/A`)
    Angular velocity bias y
* `imu_angular_vel_z_offset` (`double`, default: `N/A`)
    Angular velocity bias z
