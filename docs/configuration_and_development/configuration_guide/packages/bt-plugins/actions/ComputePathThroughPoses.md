# ComputePathThroughPoses { #compute-path-through-poses }

Invokes the ComputePathThroughPoses ROS 2 action server, which is implemented by the [nav2_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_planner) module.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/compute_path_through_poses_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
