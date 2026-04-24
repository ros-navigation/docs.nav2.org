# CancelFollowObject { #cancel-follow-object }

Used to cancel the goals given to the follow object action server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/follow_object_cancel_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
