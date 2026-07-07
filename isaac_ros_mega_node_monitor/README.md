# Isaac ROS Mega Node Monitor

## Overview

The Isaac ROS Mega Node Monitor provides a ROS 2 service to monitor the status of both regular and lifecycle nodes. It is particularly useful for:
- Verifying navigation stack components are running
- Checking system health
- Debugging node startup issues

## Usage

### 1. Launch the Monitor

Launch the monitor service:
```bash
ros2 launch isaac_ros_mega_node_monitor isaac_ros_mega_node_monitor.launch.py
```

### 2. Check Node Status

Call the service to check node status:
```bash
ros2 service call /check_nodes_alive std_srvs/srv/Trigger
```

### Response Format

The service returns a JSON-formatted response with two fields:

1. `success` (boolean):
   - `true` if all monitored nodes are alive and active
   - `false` if any nodes are dead or inactive

2. `message` (JSON string):
   ```json
   {
     "status": "healthy",  // or "unhealthy"
     "dead_nodes": [],  // list of nodes that are not running
     "inactive_lifecycle_nodes": [],  // list of lifecycle nodes not in active state
     "alive_nodes": []  // list of healthy nodes
   }
   ```

Example responses:
```json
success: False
message: {
  "status": "unhealthy",
  "dead_nodes": [
    "front_3d_lidar/flatscan_to_laserscan",
    "front_3d_lidar/pointcloud_to_flatscan",
    "global_costmap/global_costmap",
    "local_costmap/local_costmap"
  ],
  "inactive_lifecycle_nodes": [
    "behavior_server",
    "bt_navigator",
    "controller_server",
    "planner_server",
    "smoother_server",
    "velocity_smoother",
    "waypoint_follower"
  ],
  "alive_nodes": [
    "navigation_container",
    "nova_container"
  ]
}
```

### Configuration

The nodes to monitor are configured in:
`config/node_monitor_config.yaml`

```yaml
mega_node_monitor_service:
  ros__parameters:
    # Regular nodes to monitor
    monitored_nodes:
      - "front_3d_lidar/flatscan_to_laserscan"
      - "front_3d_lidar/pointcloud_to_flatscan"
      - "navigation_container"
      - "nova_container"

    # Lifecycle nodes to monitor
    monitored_lifecycle_nodes:
      - "behavior_server"
      - "bt_navigator"
      - "controller_server"
      - "planner_server"
      - "smoother_server"
```
