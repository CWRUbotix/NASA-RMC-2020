# bagging_node

A wrapper node that calls `rosbag record` to automatically record robot data to timestamped bags

## API

### Parameters
* `~topics` (list)
    * List of topics the bag will subscribe to

## Usage

#### Launch files
Launched from: `bagging_node.launch`

#### Parameters
In `bagging_node.launch`, `bag_topics.yaml` is loaded.

## Implementation Details

Uses python `subprocess` module to call `rosbag record`.\
Bags are saved with respect to `ROS_LOG_DIR`.\
glenn_bags has the same parent directory as ROS's log file.\
If no `ROS_LOG_DIR` is set will default to `$HOME/glenn_bags`.\
In this folder, bags are put into folders by date with format `%Y_%m_%d`.  
Bags are named with timestamp of creation, in format `%Y_%m_%d_%H_%M_%S`.
