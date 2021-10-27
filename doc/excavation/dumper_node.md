# dumper_node

Has an action server that takes requests to depost the material stored in the
regolith bucket. Dumps the regolith, then does a minilower and another dump to
shake out any left over.

## API
This node provides an implementation of the `SimpleActionServer` (see `actionlib` documentation),
hat takes in goals containing `actionlib/TestGoal` messages.

### Action Subscribed Topics
* `dump/goal` (`actionlib/TestActionGoal`)
  * Triggers a deposition. The value of the goal controls the number
  of times to mini-lower
* `dump/cancel` (`actionlib_msgs/GoalID`)
  * A request to cancel a specific goal.

### Action Published Topics
* `dump/feedback` (`actionlib/TestActionFeedback`)
  * Feedback, unused.
* `dump/status` (`actionlib_msgs/GoalStatusArray`)
  * Provides status information on the goals that are sent to the dump action.
* `dump/result` (`mactionlib/TestActionResult`)
  * Result is empty for the dump action.

### Subscribed Topics
* `dumper/top_limit_switch` (`std_msgs/Bool`)
  * State of the limit switch for when the bucket is at the top of the conveyor
* `dumper/weight` (`std_msgs/Float32`)
  * Data fromt the weight sensor that detects bucket regolith weight
  in the stowed position
* `dumper/position` (`std_msgs/Float32`)
  * Encoder value for deposition bucket lift

### Published Topics
* `dumper/motor_cmd` (`hwctrl/MotorCmd`)
  * The motor command for the deposition lift

### Services
TODO

### Parameters
* `~move_speed` (`double`, default: `0.5`)
  * The speed at which to command the bucket lift
* `~wait_time` (`double`, default: `2`)
  * How long the bucket stays in the tipped over position
* `~max_accel` (`double`, default: `4`)
  * Maximum bucket acceleration
* `~mini_lower_time` (`double`, default: `3`)
  * How long to execute the minilower manuever before dumping again