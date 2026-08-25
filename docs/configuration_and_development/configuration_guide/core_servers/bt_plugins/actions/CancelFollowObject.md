# CancelFollowObject { #cancel-follow-object }

Used to cancel the goals given to the follow object action server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/follow_object_cancel_node.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
