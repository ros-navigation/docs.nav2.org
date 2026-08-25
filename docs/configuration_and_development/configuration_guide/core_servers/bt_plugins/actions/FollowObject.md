# FollowObject { #follow-object }

Invokes the FollowObject ROS 2 action server, it will dynamically follow an object while maintaining a defined distance.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/follow_object_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
