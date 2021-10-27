### transit_node.py
Path planner and follower. Receives robot odometry, occupancy grid, goal position, and outputs velocity commands to drive to target position.
#### Published topics
* /glenn_base/cmd_vel
* transit_path
* transit_control_data
#### Subscribed Topics
* glenn_base/odom_map
* global_occupancy_grid