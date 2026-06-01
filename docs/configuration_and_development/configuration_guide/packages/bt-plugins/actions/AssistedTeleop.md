# AssistedTeleop { #assisted-teleop }

Invokes the AssistedTeleop ROS 2 action server, which filters teleop twist commands to prevent
collisions. This is used in nav2 Behavior Trees as a recovery behavior or a regular behavior.
The [nav2_behaviors](https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors) module implements the AssistedTeleop action server.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/assisted_teleop_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
