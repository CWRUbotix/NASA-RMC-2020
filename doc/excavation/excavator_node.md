# excavator_node

Has an action server that takes requests to excavate regolith.
Excavation continues until the deposition bucket is full.

##  API
This node provides an implementation of the `SimpleActionServer` (see `actionlib` documentation),
that takes in goals containing `actionlib/TestGoal` messages.

##### Action Subscribed Topics
* `dig/goal` (`actionlib/TestActionGoal`)
  * Triggers a excavation. The value of the goal is passed to
    the dump action server during the state where it dumps
    the top layer of regolith in the bucket to make room for ice
* `dig/cancel` (`actionlib_msgs/GoalID`)
  * A request to cancel a specific goal.

##### Action Published Topics
* `dig/feedback` (`actionlib/TestActionFeedback`)
  * Feedback, unused.
* `dig/status` (`actionlib_msgs/GoalStatusArray`)
  * Provides status information on the goals that are sent to the dig action.
* `dig/result` (`mactionlib/TestActionResult`)
  * Result is empty for the dig action.

#### Subscribed Topics
* `excavation/angle` (`std_msgs/Float32`)
  * The encoder measurement for the excavation lift angle
* `excavation/depth` (`std_msgs/Float32`)
  * Encoder measurement for excavation depth extension
* `dumper/weight` (`std_msgs/Float32`)
  * Data fromt the weight sensor that detects bucket regolith weight in the stowed position

#### Published Topics
* `excavation/angle_cmd` (`hwctrl/MotorCmd`)
  * The motor command for excavation arm lift
* `excavation/depth_cmd` (`hwctrl/MotorCmd`)
  * The motor command that controls the excavator extension to a depth
* `excavation/conveyor_cmd` (`hwctrl/MotorCmd`)
  * The motor command for excavation bucket conveyor
