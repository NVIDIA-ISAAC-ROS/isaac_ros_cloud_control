# Isaac ROS JSON Info Generator

## Overview

The `isaac_ros_json_info_generator` package has the `JsonInfoGeneratorNode` that collects data/messages from an arbitrary number of topics (of any defined ROS message type). It converts the collected messages into a JSON string, stores that string in a `std_msgs/String`, and publishes that ROS 2 string to `/info`, where the mission client (i.e. `isaac_ros_vda5050_client`) will collect and then relay that information to the mission dispatcher. The node also subscribes to the topic `/order_id` (`std_msgs/String`) and stores that as the current `order_id`; if the new `order_id` value is different from the one stored, it will clear the messages it collected and start collecting messages again.

## Launch Scripts

### `isaac_ros_json_info_generator` Launch Script

This launch script under the `launch` directory brings up the `JsonInfoGenerator` node. This launch script uses a parameter `YAML` file under the `config` folder in `json_info_generator_params.yaml`. These are the following parameters that users can configure:
| Parameter                 | Description                                                                                                                                                                                             | Default Value    |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------- |
| ros_subscriber_types      | The message types for each topic the user wants the node to subscribe to                                                                                                                                | [] (empty array) |
| ros_subscriber_topics     | The topic names for each topic the user wants the node to subscribe to. *Note*: Each element in this parameter's array corresponds to the element of the same index in `ros_subscriber_types`           | [] (empty array) |
| ros_pub_sub_queue         | The queue value used for the publishers and subscribers generated in this node.                                                                                                                         | 10               |
| messages_aggregated_count | if `messages_aggregated` is set to `True`, this value sets the number of messages to store for each topic before removing the old messages. The messages will be removed in a first-in-first-out order. | 10               |
| update_period             | How often the node should publish the messages collected (in seconds)                                                                                                                                   | 1                |

## Usage

After building this package, you can executable this node by running:

```bash
ros2 launch isaac_ros_json_info_generator isaac_ros_json_info_generator.launch.py
```

You can modify the parameter values by changing the values in `config/json_info_generator_params.yaml`.
