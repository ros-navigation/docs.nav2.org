# Spin { #spin }

Invokes the Spin ROS 2 action server, which is implemented by the [nav2_behaviors](https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors) module.
It performs an in-place rotation by a given angle.
This action is used in nav2 Behavior Trees as a recovery behavior.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/spin_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
