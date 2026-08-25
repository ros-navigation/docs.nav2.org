# BackUp { #back-up }

Invokes the BackUp ROS 2 action server, which causes the robot to back up by a specific displacement.
It performs an linear translation by a given distance.
This is used in nav2 Behavior Trees as a recovery behavior. The nav2_behaviors module implements the BackUp action server.

<!-- nav2_behaviors_: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behaviors -->

{{ render_bt_node_ports(page.title) }}

## Example


{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/back_up_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
