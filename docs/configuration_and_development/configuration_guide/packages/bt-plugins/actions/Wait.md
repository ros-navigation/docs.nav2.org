# Wait { #wait }

Invokes the Wait ROS 2 action server, which is implemented by the [nav2_behaviors](https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors) module.
This action is used in nav2 Behavior Trees as a recovery behavior.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/wait_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
