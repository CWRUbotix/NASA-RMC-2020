Copy the following template when writing a readme for a node. Remove these helpful notes in the final product.

# example_node

Put a brief description (1-5 sentences) of the node here.


## API

If your node does not use any of the following sections, no need to include them
For all of these, try to match the order here with the order they are created in the node

### Subscribed Topics

* `cmd_vel` (`geometry_msgs/Twist`)
  * Brief description of the topic (1-2) sentences
* `~data/in` (`std_msgs/String`)
  * This is how you would indicate a privately scoped topic
* `/more_data` (`std_msgs/Bool`)
  * This is how you would indicate a globally scoped topic

### Published Topics

* `data/out` (`std_msgs/String`)
    * Brief description of the topic (1-2) sentences
* `~more_data_out` (`std_msgs/Float32`)
    * This is how you would indicate a privately scoped topic

### Services
* `~do_thing` (`std_srvs/Trigger`)
  * Brief description of what the service does (1-3 sentences)
* `/another_service (`sensor_msgs/SetCameraInfo`)
  * Brief description of what the service does (1-3 sentences)

### Parameters
* `~debug` (bool, default: false)
  * Brief description here (1-4 sentences)
* `~controller_frequency`(double, default: 20.0)
  * Brief description here (1-4 sentences)


## Usage
Fill out the following sections of how this node is used in our stack.
If your node does not use some of the following sections, no need to include them.

#### Mappings
Is the node named something different than its file would imply?  
Are more than one of this node launched, all with different names?  
Are the topics listed above remapped to something else?  
Is this node in a namespace?  
List all of that here. (Delete this example text)

Two of this node are launched, one for front and one for rear, named `/front/example_node` and `/rear/example_node`
respectively.

`cmd_vel` is remapped to `/glenn_base/cmd_vel`

#### Launch files
Launched from: `launch_file.launch` and `other_launch_file.launch`,
if param `param` is set to value

#### Parameters
In `file.launch`, `param_file.yaml` is loaded. Params `a` and `b` are overridden via `<param>` tags in
`other_launch_file.launch`

#### Connections
List of where the data on all the topics this node subscribes/puslishes to comes from. This section should match
the Subscribed Topics and Published Topics section.

Please use the name of the node when it is running, followed by the type of the node. If the names match,
this is not necessary. Include if they don't match, either because it is named poorly, or more than one is spawned.
For example:

_Incoming_

* `cmd_vel` <- `base_node`, `another_node_name (node_name.cpp)`
* `~data/in` <-  `data_sender_front (data_sender_node.py)`, `data_sender_back (data_sender_node.py)`
* `/more_data` <- `base_node`

_Outgoing_
* `data/out` -> `data_reciever_node`, `another_node`
* `~more_data_out` -> `another_node`

## Implementation Details

In here, you can write paragraphs and paragraphs about how the node works, the design process,
include as many pictures as you want, etc. Pictures go in the media folder
