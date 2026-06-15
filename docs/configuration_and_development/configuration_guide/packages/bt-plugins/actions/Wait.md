# Wait { #wait }

Invokes the Wait ROS 2 action server, which is implemented by the [nav2_behaviors](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_behaviors) module.
This action is used in nav2 Behavior Trees as a recovery behavior.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/wait_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
