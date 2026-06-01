# NavigateThroughPoses { #navigate-through-poses }

Invokes the NavigateThroughPoses ROS 2 action server, which is implemented by the [bt_navigator](https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator) module.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/navigate_through_poses_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
