# robot_state_machine_node

This node does high level planning of competition tasks. It tells the robot when to dig, to deposit, where to drive.

## States

### CALIBRATE

This state runs first. The robot sits still while it localizes itself and potential calibrates other sensors
TODO: Currently doesn't do anything.

**outcomes**
* `succeeded`: Calibration only returns success

**output_keys**
* `direction_out`: Which direction to drive after calibration. Always towards excavation

**transitions**
* `succeeded` -> `DRIVE`: We drive to excavate regolith after calibration

**remapping**
* `direction_out` -> `sm_drive_direction`: So it can be used by other states

### DRIVE

An instance of `smach_ros.SimpleActionState`. Calls `move_base` action server (`move_base_msgs/MoveBaseAction`).

**outcomes**
* `succeeded_dig`: After successfully driving to excavation
* `succeeded_dump`: After successfully driving to deposition

**input_keys**
* `direction_in`: Are we driving to excavation or deposition?
* `sub_run`: Every excavation needs to be in a different location. This is based on ow many runs we have done.

**transitions**
* `succeeded_dig` -> `DIG`: After driving to excavation, we excavate
* `succeeded_dump` -> `DUMP`: After driving to the deposition zone, we deposit

**remapping**
* `direction_in` -> `sm_drive_direction`: So it knows which way to go

### DIG

An instance of `smach_ros.SimpleActionState`. Calls `dig` action server (`actionlib_msgs/TestAction`).

**outcomes**
* `succeeded`: After successfully excavating

**output_keys**
* `direction_out`: Which direction to drive after excavation. Always towards deposition

**transitions**
* `succeeded` -> `DRIVE`: After excavating, we drive to the deposition zone

**remapping**
* `direction_out` -> `sm_drive_direction`: So it can be used by other states

### DUMP

An instance of `smach_ros.SimpleActionState`. Calls `dump` action server (`actionlib_msgs/TestAction`).

**outcomes**
* `succeeded`: After successfully depositing

**output_keys**
* `direction_out`: After deposition, always drive to excavation
* `sub_run`: Passes this on to the `DRIVE` state so it knows how much it has dug so far

**transitions**
* `succeeded` -> `DRIVE`: After deposition, we drive to the excavation zone

**remapping**
* `direction_out` -> `sm_drive_direction`: So it can be used by other states
