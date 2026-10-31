> [!IMPORTANT]
> This is a proof-of-concept project. It shows MBF's interfaces match with nav2 by simple forwarding actions for planning or control from MBF to nav2. In the section 'How to solve this properly' we describe how we are planning to integrate nav2 layered costmap into MBF properly.

# MBF Nav2 Connector

This package provides a bridge between Move Base Flex and Nav2 navigation stack. Instead of implementing planners and controllers from scratch, it acts as a connector that forwards:
- MBF planning requests to Nav2's `ComputePathToPose` action
- MBF control requests to Nav2's `FollowPath` action

This allows you to use the MBF framework and interfaces while leveraging Nav2's navigation capabilities.

## Features

- **Easy integration**: Simply configure MBF to use these plugins and point to your Nav2 action servers
- **Leverage existing Nav2 infrastructure**: Use Nav2's well-tested planners and controllers
- **MBF flexibility**: Benefit from MBF's map-agnostic architecture. Switch seemlesly between, e.g. mesh-based and costmap navigation

## Dependencies

- ROS 2 (tested on Humble/Iron)
- Move Base Flex (mbf_abstract_core, mbf_msgs, mbf_utility)
- Nav2 (nav2_msgs, nav2_core)
- Standard ROS 2 packages (rclcpp, geometry_msgs, nav_msgs, tf2)

## Building

```bash
cd ~/mbf_nav2_ws
colcon build --packages-select mbf_nav2_connector
source install/local_setup.bash
```

## Usage

### Prerequisites

Make sure Nav2 navigation stack is running.

### Configure MBF

The package includes an example configuration in `config/mbf_nav2_connector.yaml`:

```yaml
mbf_nav2:
  ros__parameters:
    planners: ["nav2_planner"]
    controllers: ["nav2_controller"]
    
    # Nav2 Planner Plugin
    nav2_planner:
      type: "mbf_nav2_connector::Nav2Planner"
      nav2_planner_action: "compute_path_to_pose"
      nav2_planner_name: "GridBased" # this is the connection to the actual plugin
      action_timeout: 30.0
    
    # Nav2 Controller Plugin
    nav2_controller:
      type: "mbf_nav2_connector::Nav2Controller"
      nav2_controller_action: "follow_path"
      nav2_controller_name: "FollowPath" # this is the connection to the actual plugin
      action_timeout: 60.0
```

Make sure the nav2 actions and plugin names exist.

### Launch MBF Connector

```bash
ros2 launch mbf_nav2_connector mbf_nav2_connector.launch.py
```

## Architecture

```
MBF Actions → Nav2Planner/Nav2Controller Plugins → Nav2 Action Servers
```

1. MBF receives a navigation request (get_path, exe_path, or move_base)
2. MBF calls the configured planner/controller plugin
3. The plugin converts the request and forwards it to Nav2's action servers
4. Nav2 processes the request using its configured planners/controllers
5. Results are converted back and returned to MBF
6. MBF returns the result to the original caller

## Limitations

- Nav2 must be running and configured separately
- Additional layer of abstraction may add slight latency
- Some Nav2-specific features might not map directly to MBF interfaces
- The controller plugin returns zero velocities since Nav2 handles velocity commands internally

## How to solve this properly

This repo simply connects MBF to Nav2 by forwarding action calls. This results in the limitations that were already known before starting this project. It is not intended to be the final solution, but rather a quickly developed proof of concept to demonstrate the potential compatibility between MBF and Nav2.
Our actual goal is to implement `mbf_nav2` in a way that follows the concepts of [MeshNav](https://github.com/naturerobots/mesh_navigation):

* Maintain the Nav2 costmap similarly to the MeshMap
* Load the existing Nav2 plugins for planning and control

This approach would most closely mirror what MeshNav does and would provide full compatibility with Nav2 planners and controllers. Once this is completed, we will have identical interfaces for MeshNav and Nav2, enabling easier switching between maps, planner strategies, and controller strategies depending on the environment or overall [context](https://ieeexplore.ieee.org/abstract/document/10812052).

Contributions are welcome. If you are interested in helping implement this, feel free to reach out. It is possible that we are already working on it in the background.
