# FollowPath { #follow-path }

Invokes the FollowPath ROS 2 action server, which is implemented by the controller plugin modules loaded.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/follow_path_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
