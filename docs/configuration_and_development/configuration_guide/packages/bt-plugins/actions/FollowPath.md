# FollowPath { #follow-path }

Invokes the FollowPath ROS 2 action server, which is implemented by the controller plugin modules loaded.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/follow_path_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
