# BackUp { #back-up }

Invokes the BackUp ROS 2 action server, which causes the robot to back up by a specific displacement.
It performs an linear translation by a given distance.
This is used in nav2 Behavior Trees as a recovery behavior. The nav2_behaviors module implements the BackUp action server.

<!-- nav2_behaviors_: https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors -->

{{ render_bt_node_ports(page.title) }}

## Example


{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/back_up_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
