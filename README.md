# NASA-RMC-2020

CWRUbotix Coding Conventions

## ROS 
Taken from http://wiki.ros.org/ROS/Patterns/Conventions

### Naming

In general all names should be descriptive. The following are examples of the casing for different names.

* Package names: package_name
* Topics/Services: topic_or_service_name
* Message/Service file names: MyMessage.msg, MyService.srv
  * Message/Service fields: message_field
  * Message/Service constant values: CONSTANT_NAME
* Node names: my_node
* Launch files: launch_file.launch

### Topics, services, actions
See https://wiki.ros.org/ROS/Patterns/Communication#Communication_via_Topics_vs_Services_vs_X

* Topics are for things that are published repeatedly
* Services are for when you want to cause something to happen
* Actions are for the same cases as services but when the thing you want to happen might take a while or need to be interrupted

### General things

* Use agreed upon ROS topic names. For example, the topic to drive the robot should be cmd_vel and should be of type geometry_msgs/Twist
* Use existing ROS topic types when applicable. For example sensor_msgs/BatteryState or sensor_msgs/Imu
* Use namespacing in the parameter server so that everything isn't in the global namespace. For example autonomy/target_robot_speed instead of target_robot_speed

## CWRUbotix Custom Conventions

* Python ROS nodes should be implemented as classes
  * Anything being published could have its own publish function that encapsulates the code to convert data to ROS msg types
