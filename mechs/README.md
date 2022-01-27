#  Package: excavation

Nodes that run state machines that control the operation of excavation and deposition. When activated they run once cycle of excvation or deposition.
Uses actionlib.

## Nodes

### dumper_node.py

Has an action server that takes requests to depost the material stored in the
regolith bucket. Dumps the regolith, then does a minilower and another dump to
shake out any left over.

#### Action API
The dumper_node.py provides an implementation of the SimpleActionServer (see actionlib documentation), that takes in goals containing actionlib/TestGoal messages.

##### Action Subscribed Topics
* `dump/goal` (`actionlib/TestActionGoal`)
    Triggers a deposition. The value of the goal controls the number
    of times to mini-lower
* `dump/cancel` (`actionlib_msgs/GoalID`)
    A request to cancel a specific goal.

##### Action Published Topics
`dump/feedback` (`actionlib/TestActionFeedback`)
    Feedback, unused.
`dump/status` (`actionlib_msgs/GoalStatusArray`)
    Provides status information on the goals that are sent to the dump action.
`dump/result` (`mactionlib/TestActionResult`)
    Result is empty for the dump action.

#### Published Topics
* `dumper/motor_cmd` (`hwctrl/MotorCmd`)
    The motor command for the deposition lift

#### Subscribed Topics
* `dumper/top_limit_switch` (`std_msgs/Bool`)
    State of the limit switch for when the bucket is at the top of the conveyor
* `dumper/weight` (`std_msgs/Float32`)
    Data fromt the weight sensor that detects bucket regolith weight
    in the stowed position
* `dumper/position` (`std_msgs/Float32`)
    Encoder value for deposition bucket lift

#### Services
* `move_open_loop` (`open_loop_move/OpenLoopMove`)
    Used to trigger the move

#### Parameters
* `~move_speed` (`double`, default: `N/A`)
    The speed at which to command the bucket lift
* `~wait_time` (`double`, default: `N/A`)
    How long the bucket stays in the tipped over position
* `~max_accel` (`double`, default: `N/A`)
    Maximum bucket acceleration
* `~mini_lower_time` (`double`, default: `N/A`)
    How long to execute the minilower manuever before dumping again


### excavator_node.py

Has an action server that takes requests to excavate regolith.
Excavation continues until the deposition bucket is full.

#### Action API
The excavator_node.py provides an implementation of the SimpleActionServer (see actionlib documentation), that takes in goals containing actionlib/TestGoal messages.

##### Action Subscribed Topics
* `dig/goal` (`actionlib/TestActionGoal`)
    Triggers a excavation. The value of the goal is passed to
    the dump action server during the state where it dumps
    the top layer of regolith in the bucket to make room for ice
* `dig/cancel` (`actionlib_msgs/GoalID`)
    A request to cancel a specific goal.

##### Action Published Topics
`dig/feedback` (`actionlib/TestActionFeedback`)
    Feedback, unused.
`dig/status` (`actionlib_msgs/GoalStatusArray`)
    Provides status information on the goals that are sent to the dig action.
`dig/result` (`mactionlib/TestActionResult`)
    Result is empty for the dig action.

#### Published Topics
* `excavation/angle_cmd` (`hwctrl/MotorCmd`)
    The motor command for excavation arm lift
* `excavation/depth_cmd` (`hwctrl/MotorCmd`)
    The motor command that controls the excavator extension to a depth
* `excavation/conveyor_cmd` (`hwctrl/MotorCmd`)
    The motor command for excavation bucket conveyor

#### Subscribed Topics
* `excavation/angle` (`std_msgs/Float32`)
    The encoder measurement for the excavation lift angle
* `excavation/depth` (`std_msgs/Float32`)
    Encoder measurement for excavation depth extension
* `dumper/weight` (`std_msgs/Float32`)
    Data fromt the weight sensor that detects bucket regolith weight
    in the stowed position
